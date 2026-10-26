#include "debug_uart.h"
#include "usart.h"
#include "board_config.h"
#include "imu.h"
#include "motor.h"
#include "speed_control.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_PERIOD_MS 5000U
#define VECTOR_FORMAT "%ld,%ld,%ld,%ld"
#define VECTOR_VALUES(a,scale) fixed((a)[0],scale),fixed((a)[1],scale),fixed((a)[2],scale),fixed((a)[3],scale)
/* 1024 bytes at 115200/8N1 take <89 ms. Never overwrite an active IT buffer. */
static char buffer[1025];
static uint32_t last_publish,last_config,sequence,dropped,tx_errors,bad_numbers;
static uint8_t config_pending;
static struct {
    uint32_t last_ms,interval,min_interval,max_interval,max_cost,ticks;
    uint8_t valid_mask;
    float speed_min[MOTOR_COUNT],speed_max[MOTOR_COUNT];
} control;

static long fixed(float value,float scale)
{
    const float scaled=value*scale;
    if (!isfinite(scaled)) { ++bad_numbers; return 0; }
    if (scaled>=2147483648.0f) { ++bad_numbers; return INT32_MAX; }
    if (scaled<-2147483648.0f) { ++bad_numbers; return INT32_MIN; }
    if (scaled==-2147483648.0f) return INT32_MIN;
    return (long)lroundf(scaled);
}
void DebugUart_Init(void)
{
    const uint32_t now=HAL_GetTick();
    last_publish=now-DEBUG_PERIOD_MS; last_config=now;
    sequence=dropped=tx_errors=bad_numbers=0;
    config_pending=1;
    memset(&control,0,sizeof(control));
}
void DebugUart_RecordControl(uint32_t now,uint32_t interval,uint32_t cost,const EncoderData *encoder)
{
    control.last_ms=now; control.interval=interval;
    if (!control.ticks || interval<control.min_interval) control.min_interval=interval;
    if (interval>control.max_interval) control.max_interval=interval;
    if (cost>control.max_cost) control.max_cost=cost;
    for (unsigned i=0;i<MOTOR_COUNT;++i) {
        const float speed=encoder->speed_mm_s[i];
        if (!isfinite(speed)) { ++bad_numbers; continue; }
        if (!(control.valid_mask&(1U<<i)) || speed<control.speed_min[i]) control.speed_min[i]=speed;
        if (!(control.valid_mask&(1U<<i)) || speed>control.speed_max[i]) control.speed_max[i]=speed;
        control.valid_mask|=(uint8_t)(1U<<i);
    }
    ++control.ticks;
}
static int format_config(uint32_t now,uint32_t seq)
{
    const int motor_sign[MOTOR_COUNT]=MOTOR_DIRECTION_SIGNS;
    const int enc_sign[MOTOR_COUNT]=ENCODER_DIRECTION_SIGNS;
    return snprintf(buffer,sizeof(buffer),
        "log=V10L1 type=cfg t=%lu seq=%lu baud=%lu period=%u "
        "wheel_um=%ld cpr=%ld motor_sign=%d,%d,%d,%d enc_sign=%d,%d,%d,%d "
        "motor_map=%u,%u,%u,%u pwm_period=%u pwm_limit=%u deadzone=%u "
        "speed_kp1000=%ld speed_ki1000=%ld speed_kd1000=%ld accel=%ld decel=%ld "
        "gray_active=%u x1_left=%u gray_settle_us=%u imu_sign=%ld "
        "heading_kp1000=%ld heading_kd1000=%ld heading_max=%ld boundary_max100=%ld "
        "trim_ppm=%ld straight=%ld finish=%ld drop=%lu txerr=%lu",
        (unsigned long)now,(unsigned long)seq,(unsigned long)huart1.Init.BaudRate,
        (unsigned)DEBUG_PERIOD_MS,fixed(WHEEL_DIAMETER_MM,1000),fixed(WHEEL_COUNTS_PER_REV,1),
        motor_sign[0],motor_sign[1],motor_sign[2],motor_sign[3],
        enc_sign[0],enc_sign[1],enc_sign[2],enc_sign[3],
        (unsigned)LEFT_FRONT_MOTOR,(unsigned)LEFT_REAR_MOTOR,
        (unsigned)RIGHT_FRONT_MOTOR,(unsigned)RIGHT_REAR_MOTOR,
        (unsigned)MOTOR_PWM_PERIOD_COUNTS,(unsigned)MOTOR_PWM_LIMIT_COUNTS,(unsigned)MOTOR_DEADZONE_COUNTS,
        fixed(SPEED_KP,1000),fixed(SPEED_KI,1000),fixed(SPEED_KD,1000),
        fixed(SPEED_ACCEL_MM_S2,1),fixed(SPEED_DECEL_MM_S2,1),
        (unsigned)GRAY8_ACTIVE_LEVEL,(unsigned)GRAY8_X1_IS_LEFT,(unsigned)GRAY8_SETTLE_US,
        fixed(IMU_YAW_SIGN,1),fixed(HEADING_STRAIGHT_KP,1000),fixed(HEADING_STRAIGHT_KD,1000),
        fixed(HEADING_STRAIGHT_MAX_MM_S,1),fixed(HEADING_BOUNDARY_MAX_DEG,100),
        fixed(ROUTE_STRAIGHT_TRIM,1000000),fixed(ROUTE_STRAIGHT_SPEED_MM_S,1),
        fixed(ROUTE_FINISH_SPEED_MM_S,1),(unsigned long)dropped,(unsigned long)tx_errors);
}
static int format_data(uint32_t now,uint32_t seq,const char *state,unsigned fault,
                       const Gray8Data *gray,const EncoderData *encoder,const int16_t *pwm,
                       const RouteState *route,const OdomData *odom,uint8_t armed)
{
    const ImuData *imu=Imu_GetData();
    const float *target=SpeedControl_GetTargets(),*ramp=SpeedControl_GetRampedTargets();
    const uint16_t *out=Motor_GetOutputs();
    unsigned raw=0;
    for (unsigned i=0;i<8U;++i) if (gray->raw[i]) raw|=1U<<i;
    float vmin[MOTOR_COUNT],vmax[MOTOR_COUNT];
    for (unsigned i=0;i<MOTOR_COUNT;++i) {
        vmin[i]=(control.valid_mask&(1U<<i))?control.speed_min[i]:encoder->speed_mm_s[i];
        vmax[i]=(control.valid_mask&(1U<<i))?control.speed_max[i]:encoder->speed_mm_s[i];
    }
    int length=snprintf(buffer,sizeof(buffer),
        "log=V10L1 type=data t=%lu seq=%lu state=%s fault=%u "
        "raw=%02X line=%02X n=%u valid=%u pos10=%ld marker=%u s=%ld "
        "cnt=%ld,%ld,%ld,%ld denc=%d,%d,%d,%d v=" VECTOR_FORMAT " v10=" VECTOR_FORMAT " "
        "target10=" VECTOR_FORMAT " ramp10=" VECTOR_FORMAT " "
        "vmin10=" VECTOR_FORMAT " vmax10=" VECTOR_FORMAT " "
        "pwm=%d,%d,%d,%d out=%u,%u,%u,%u,%u,%u,%u,%u "
        "stage=%s side=%ld,%ld seg=%ld,%ld goal=%ld,%ld cmd=%ld,%ld arm=%u "
        "imu=%u cal=%u imu_age=%lu bias1000=%ld yaw10=%ld rate10=%ld yaw100=%ld rate100=%ld "
        "ref10=%ld off10=%ld dt=%lu dtmin=%lu dtmax=%lu costmax=%lu ctrl_n=%lu ctrl_age=%lu "
        "drop=%lu txerr=%lu",
        (unsigned long)now,(unsigned long)seq,state,fault,raw,(unsigned)gray->active_mask,
        (unsigned)gray->active_count,(unsigned)gray->line_valid,fixed(gray->position,10),(unsigned)gray->marker,
        fixed(encoder->travel_mm,1),
        (long)encoder->count[0],(long)encoder->count[1],(long)encoder->count[2],(long)encoder->count[3],
        (int)encoder->delta[0],(int)encoder->delta[1],(int)encoder->delta[2],(int)encoder->delta[3],
        VECTOR_VALUES(encoder->speed_mm_s,1),VECTOR_VALUES(encoder->speed_mm_s,10),
        VECTOR_VALUES(target,10),VECTOR_VALUES(ramp,10),VECTOR_VALUES(vmin,10),VECTOR_VALUES(vmax,10),
        (int)pwm[0],(int)pwm[1],(int)pwm[2],(int)pwm[3],
        (unsigned)out[0],(unsigned)out[1],(unsigned)out[2],(unsigned)out[3],
        (unsigned)out[4],(unsigned)out[5],(unsigned)out[6],(unsigned)out[7],
        Route_SegmentName(route),fixed(odom->left_mm,1),fixed(odom->right_mm,1),
        fixed(route->progress_left_mm,1),fixed(route->progress_right_mm,1),
        fixed(route->goal_left_mm,1),fixed(route->goal_right_mm,1),
        fixed(route->command_left_mm_s,1),fixed(route->command_right_mm_s,1),(unsigned)armed,
        (unsigned)imu->status,imu->calibration_samples,(unsigned long)(now-imu->last_sample_ms),
        fixed(imu->bias_z_dps,1000),fixed(imu->yaw_deg,10),fixed(imu->yaw_rate_dps,10),
        fixed(imu->yaw_deg,100),fixed(imu->yaw_rate_dps,100),
        fixed(route->heading_target_deg,10),fixed(route->boundary_offset_deg,10),
        (unsigned long)control.interval,(unsigned long)control.min_interval,(unsigned long)control.max_interval,
        (unsigned long)control.max_cost,(unsigned long)control.ticks,
        (unsigned long)(now-control.last_ms),(unsigned long)dropped,(unsigned long)tx_errors);
    if (length>0 && (unsigned)length<sizeof(buffer)) {
        int tail=snprintf(buffer+length,sizeof(buffer)-(unsigned)length," badnum=%lu",(unsigned long)bad_numbers);
        if (tail<0) return tail;
        length+=tail;
    }
    return length;
}
static uint16_t checksum(const char *data,unsigned length)
{
    uint16_t crc=0xFFFFU;
    for (unsigned i=0;i<length;++i) {
        crc^=(uint16_t)((uint16_t)(uint8_t)data[i]<<8);
        for (unsigned bit=0;bit<8U;++bit)
            crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1)^0x1021U:(uint32_t)crc<<1);
    }
    return crc;
}
void DebugUart_Publish(uint32_t now,const char *state,unsigned fault,const Gray8Data *gray,
                       const EncoderData *encoder,const int16_t *pwm,const RouteState *route,
                       const OdomData *odom,uint8_t finish_armed)
{
    if ((uint32_t)(now-last_publish)<DEBUG_PERIOD_MS) return;
    last_publish=now;
    const uint32_t seq=sequence++;
    if (huart1.gState!=HAL_UART_STATE_READY) { ++dropped; return; }
    const uint8_t config=config_pending || (uint32_t)(now-last_config)>=CONFIG_PERIOD_MS;
    int length=config?format_config(now,seq):
        format_data(now,seq,state,fault,gray,encoder,pwm,route,odom,finish_armed);
    uint8_t overflow=0;
    if (length<=0 || (unsigned)length+12U>=sizeof(buffer)) {
        overflow=1;
        ++dropped;
        length=snprintf(buffer,sizeof(buffer),
            "log=V10L1 type=overflow t=%lu seq=%lu wanted=%d drop=%lu txerr=%lu",
            (unsigned long)now,(unsigned long)seq,length,(unsigned long)dropped,(unsigned long)tx_errors);
        if (length<=0 || (unsigned)length>=sizeof(buffer)) return;
    }
    const unsigned crc=checksum(buffer,(unsigned)length);
    const int tail=snprintf(buffer+length,sizeof(buffer)-(unsigned)length," crc=%04X\r\n",crc);
    if (tail<=0 || (unsigned)(length+tail)>=sizeof(buffer)) { ++dropped; return; }
    length+=tail;
    if (HAL_UART_Transmit_IT(&huart1,(uint8_t *)buffer,(uint16_t)length)!=HAL_OK) {
        ++tx_errors; ++dropped; return;
    }
    if (!overflow) {
        if (config) { last_config=now; config_pending=0; }
        else {
            control.ticks=0; control.valid_mask=0;
            control.min_interval=control.max_interval=control.max_cost=0;
        }
    }
}
