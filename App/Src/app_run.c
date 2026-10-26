#include "app_run.h"
#include "main.h"
#include "board_config.h"
#include "motor.h"
#include "encoder.h"
#include "gray8.h"
#include "speed_control.h"
#include "chassis.h"
#include "route.h"
#include "crossline_stop.h"
#include "key.h"
#include "debug_uart.h"
#include "hmi.h"
#include "imu.h"
#include <math.h>
#include <string.h>

typedef enum { APP_IDLE, APP_RUNNING, APP_BRAKING, APP_DONE, APP_FAULT } AppState;
typedef enum {
    /* Encoder fault codes 3/4 remain reserved for UART compatibility. */
    FAULT_NONE, FAULT_NO_START_LINE, FAULT_LINE_LOST, FAULT_ENCODER_STALL,
    FAULT_ENCODER_REVERSE, FAULT_CONTROL_LATE, FAULT_RUN_TIMEOUT,
    FAULT_BRAKE_TIMEOUT, FAULT_MARKER_STUCK,
    FAULT_SEGMENT_TIMEOUT, FAULT_FINISH_MISSING, FAULT_ROUTE_CONFIG,
    FAULT_IMU_NOT_READY, FAULT_IMU_LOST
} FaultCode;
static AppState state;
static FaultCode fault;
static Gray8Data gray;
static RouteState route;
static OdomData odom;
static CrosslineStop crossline;
static uint32_t last_control, run_start, braking_start, settled_ms;
static uint32_t beep_start, beep_duration, brake_hold_start;
static uint8_t brake_hold;

static const char *state_name(void)
{
    switch (state) {
    case APP_IDLE:
        if (Imu_GetData()->status==IMU_CALIBRATING) return "CAL";
        if (!Imu_DataReady(Imu_GetData(),HAL_GetTick())) return "IMU_ERR";
        return "IDLE";
    case APP_RUNNING: return "RUN";
    case APP_BRAKING: return "BRAKE";
    case APP_DONE: return "DONE";
    default: return "FAULT";
    }
}
static void beep(uint32_t now, uint32_t duration)
{
    beep_start=now; beep_duration=duration;
}
static void stop_as(AppState next_state, FaultCode reason, uint32_t now)
{
    SpeedControl_Stop(1);
    state=next_state; fault=reason;
    brake_hold=1; brake_hold_start=now;
    beep(now, next_state==APP_FAULT ? 600U : 150U);
    HMI_Stop(now);
}
static uint8_t moving(void) { return state==APP_RUNNING || state==APP_BRAKING; }

static void start_run(uint32_t now)
{
    SpeedControl_Stop(0);
    brake_hold=0;
    if (!Imu_DataReady(Imu_GetData(),now)) {
        stop_as(APP_FAULT,FAULT_IMU_NOT_READY,now);
        return;
    }
    Imu_ResetYaw();
    Encoder_ResetTrip();
    Odometry_FromEncoder(Encoder_GetData(),&odom);
    Route_Start(&route,&odom,now,Imu_GetData());
    Crossline_Reset(&crossline);
    if (route.result!=ROUTE_RUNNING) {
        stop_as(APP_FAULT,route.result==ROUTE_IMU_ERROR?FAULT_IMU_NOT_READY:FAULT_ROUTE_CONFIG,now);
        return;
    }
    run_start=now; settled_ms=0;
    fault=FAULT_NONE; state=APP_RUNNING; last_control=now;
    beep(now,80U);
    HMI_Start(now);
}

static void control_tick(uint32_t now, uint32_t interval)
{
    const EncoderData *encoder;
    Encoder_Update(interval);
    if (state==APP_RUNNING && Route_InArc(&route))
        memset(&gray,0,sizeof(gray)); /* Arc: no scan, steering or bar detection. */
    else Gray8_ReadAll(&gray);
    encoder=Encoder_GetData();
    Odometry_FromEncoder(encoder,&odom);
    if (moving() && interval > CONTROL_MAX_INTERVAL_MS) {
        stop_as(APP_FAULT,FAULT_CONTROL_LATE,now);
        return;
    }
    {
        uint8_t stationary=1;
        for (unsigned i=0;i<MOTOR_COUNT;++i)
            if (fabsf(encoder->speed_mm_s[i])>2.0f) stationary=0;
        Imu_Tick(now,stationary && !moving());
    }
    if (state==APP_RUNNING) {
        RouteResult result;
        if (!Imu_DataReady(Imu_GetData(),now)) {
            stop_as(APP_FAULT,FAULT_IMU_LOST,now);
            return;
        }
        uint8_t finish_detected;
        if ((uint32_t)(now-run_start) >= ROUTE_MAX_RUN_MS) {
            stop_as(APP_FAULT,FAULT_RUN_TIMEOUT,now);
            return;
        }
        finish_detected=0;
        if (!Route_InArc(&route))
            finish_detected=Crossline_Update(&crossline,gray.marker,Route_FinishArmed(&route,&odom),interval);
        result=Route_Update(&route,&odom,now,finish_detected,&gray,Imu_GetData());
        if (result==ROUTE_COMPLETE) {
            state=APP_BRAKING;
            braking_start=now; settled_ms=0;
            Chassis_SetForwardTurn(0,0);
        } else {
            if (result!=ROUTE_RUNNING) {
                stop_as(APP_FAULT,result==ROUTE_TIMEOUT ? FAULT_SEGMENT_TIMEOUT :
                        (result==ROUTE_FINISH_MISSING ? FAULT_FINISH_MISSING :
                        (result==ROUTE_IMU_ERROR ? FAULT_IMU_LOST : FAULT_ROUTE_CONFIG)),now);
                return;
            }
            Chassis_SetLeftRight(route.command_left_mm_s,route.command_right_mm_s);
        }
        SpeedControl_Tick(encoder->speed_mm_s,interval);
    } else if (state==APP_BRAKING) {
        uint8_t stationary=1;
        const float *ramped;
        Chassis_SetForwardTurn(0,0);
        SpeedControl_Tick(encoder->speed_mm_s,interval);
        ramped=SpeedControl_GetRampedTargets();
        for (unsigned i=0; i<MOTOR_COUNT; ++i)
            if (fabsf(encoder->speed_mm_s[i]) > STOP_SETTLE_SPEED_MM_S || fabsf(ramped[i]) > 0.5f)
                stationary=0;
        settled_ms=stationary ? settled_ms+interval : 0;
        if (settled_ms >= STOP_SETTLE_MS) stop_as(APP_DONE,FAULT_NONE,now);
        else if ((uint32_t)(now-braking_start) >= STOP_MAX_BRAKING_MS)
            stop_as(APP_FAULT,FAULT_BRAKE_TIMEOUT,now);
    }
}

void App_Init(void)
{
    uint32_t now=HAL_GetTick();
    state=APP_IDLE; fault=FAULT_NONE;
    Motor_Init();
    Encoder_Init();
    Gray8_Init();
    SpeedControl_Init();
    memset(&route,0,sizeof(route));
    memset(&odom,0,sizeof(odom));
    Crossline_Reset(&crossline);
    brake_hold=0; beep_duration=0; settled_ms=0;
    Gray8_ReadAll(&gray);
    Key_Init(now);
    DebugUart_Init();
    HMI_Init();
    Imu_Init();
    last_control=HAL_GetTick();
}

void App_Poll(void)
{
    uint32_t now=HAL_GetTick();
    uint8_t start_pressed=Key_StartPressed(now);
    if (Key_StopHeld()) {
        if (moving()) stop_as(APP_IDLE,FAULT_NONE,now);
    } else if (start_pressed) {
        if (moving()) stop_as(APP_IDLE,FAULT_NONE,now);
        else start_run(now);
    }
    if ((uint32_t)(now-last_control) >= CONTROL_PERIOD_MS) {
        uint32_t interval=(uint32_t)(now-last_control);
        last_control=now;
        control_tick(now,interval);
        DebugUart_RecordControl(now,interval,(uint32_t)(HAL_GetTick()-now),Encoder_GetData());
    }
    if (brake_hold && !moving() && (uint32_t)(now-brake_hold_start) >= STOP_BRAKE_HOLD_MS) {
        Motor_StopAll(0); brake_hold=0;
    }
    HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,
        (uint32_t)(now-beep_start) < beep_duration ? GPIO_PIN_SET : GPIO_PIN_RESET);
    {
        uint8_t led_on;
        if (state==APP_DONE) led_on=1;
        else if (state==APP_FAULT) led_on=(now/100U)%2U==0U;
        else if (moving()) led_on=(now/250U)%2U==0U;
        else led_on=(now/1000U)%2U==0U;
        HAL_GPIO_WritePin(STATUS_LED_GPIO_Port,STATUS_LED_Pin,led_on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    DebugUart_Publish(now,state_name(),(unsigned)fault,&gray,Encoder_GetData(),SpeedControl_GetPwm(),
                      &route,&odom,Route_FinishArmed(&route,&odom));
    {
        const EncoderData *enc=Encoder_GetData();
        const float speed=(fabsf(enc->speed_mm_s[0])+fabsf(enc->speed_mm_s[1])+
                           fabsf(enc->speed_mm_s[2])+fabsf(enc->speed_mm_s[3]))*0.25f;
        HMI_Tick(now,state_name(),(uint8_t)fault,odom.center_mm,speed);
    }
}
