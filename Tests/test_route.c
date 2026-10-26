/* Offline-only fixtures: real MPU logic/route/app/speed/HMI, fake I2C/GPIO,
 * and an ideal differential-drive plant. Does not validate actual hardware. */
#include "app_run.h"
#include "main.h"
#include "board_config.h"
#include "route.h"
#include "imu.h"
#include "imu_bus.h"
#include "crossline_stop.h"
#include "debug_uart.h"
#include "speed_control.h"
#include "hmi.h"
#include "oled.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static int near(float a,float b) { return fabsf(a-b)<0.5f; }
static uint32_t clock_ms;
static EncoderData enc;
static Gray8Data sensors;
static float wheel_mm[4],plant_yaw_rate,plant_yaw,plant_yaw_scale;
static uint8_t key1,key2;
static unsigned motion_mode,reported_fault,gray_read_calls,stage_mask;
static char reported_state[16];
static RouteState reported_route;
static float heading_on_exit[4];
static uint8_t saw_reverse;
static uint8_t registers[128],bus_available,bus_fail,bus_data_ready,bus_bad_id,bus_bad_readback;
static uint8_t bus_address;
static float sample_rate_override,sample_ax,sample_ay,sample_az;
static uint8_t override_rate,drift;
static const float gyro_bias=1.25f;
static char screen_rows[8][21];
static unsigned oled_service_calls;
uint32_t HAL_GetTick(void) { return clock_ms; }
void HAL_GPIO_WritePin(unsigned port,uint16_t pin,unsigned value) { (void)port; (void)pin; (void)value; }
void ImuBus_Init(void) {}
void ImuBus_BootDelay(uint32_t ms) { clock_ms+=ms; }
static void word(uint8_t *p,int value) { const uint16_t u=(uint16_t)value; p[0]=(uint8_t)(u>>8); p[1]=(uint8_t)u; }
uint8_t ImuBus_Write(uint8_t address,uint8_t reg,uint8_t value)
{
    if (!bus_available || bus_fail || address!=bus_address || reg>=128U) return 0;
    if (reg==0x6BU && value==0x80U) { memset(registers,0,sizeof(registers)); registers[0x75]=0x68; }
    else registers[reg]=value;
    return 1;
}
uint8_t ImuBus_Read(uint8_t address,uint8_t reg,uint8_t *data,unsigned count)
{
    if (!bus_available || bus_fail || address!=bus_address || !data || reg+count>128U) return 0;
    if (reg==0x3AU && count==15U) {
        memset(data,0,count); data[0]=bus_data_ready;
        word(&data[1],(int)lroundf(sample_ax*16384));
        word(&data[3],(int)lroundf(sample_ay*16384));
        word(&data[5],(int)lroundf(sample_az*16384));
        const float raw_rate=gyro_bias-(override_rate?sample_rate_override:plant_yaw_rate);
        word(&data[13],(int)lroundf(raw_rate*65.5f));
    } else {
        memcpy(data,&registers[reg],count);
        if (reg==0x75U) data[0]=bus_bad_id?0x42U:0x68U;
        if (reg==0x1BU && bus_bad_readback) data[0]=0;
    }
    return 1;
}
void Motor_Init(void) {}
void Motor_SetPwm(unsigned id,int16_t pwm)
{
    const float ramped=SpeedControl_GetRampedTargets()[id];
    CHECK(pwm>=-800 && pwm<=800); CHECK(ramped>=0?pwm>=0:pwm<=0);
    if (ramped<0) saw_reverse=1;
}
void Motor_StopAll(uint8_t brake) { (void)brake; }
void Encoder_Init(void) { memset(&enc,0,sizeof(enc)); memset(wheel_mm,0,sizeof(wheel_mm)); plant_yaw=0; plant_yaw_rate=0; }
void Encoder_ResetTrip(void) { Encoder_Init(); }
const EncoderData *Encoder_GetData(void) { return &enc; }
void Encoder_Update(uint32_t interval)
{
    const float mm_per_count=3.14159265359f*WHEEL_DIAMETER_MM/WHEEL_COUNTS_PER_REV;
    const float *targets=SpeedControl_GetRampedTargets();
    float actual[4];
    for (unsigned i=0;i<4U;++i) {
        actual[i]=motion_mode==1U?0:targets[i];
        if (drift) actual[i]*=i<2U?0.97f:1.03f;
        if (motion_mode==4U && i==2U) actual[i]=0;
        const float reported=motion_mode==2U?-actual[i]:actual[i];
        wheel_mm[i]+=reported*(float)interval/1000;
        const int32_t count=(int32_t)(wheel_mm[i]/mm_per_count);
        enc.delta[i]=(int16_t)(count-enc.count[i]); enc.count[i]=count;
        enc.speed_mm_s[i]=(float)enc.delta[i]*mm_per_count*1000/(float)interval;
    }
    plant_yaw_rate=((actual[0]+actual[1])-(actual[2]+actual[3]))*0.5f/
        (ROUTE_EFFECTIVE_TRACK_MM*ROUTE_TURN_DIFFERENTIAL_GAIN)*57.29577951f*plant_yaw_scale;
    plant_yaw+=plant_yaw_rate*(float)interval/1000;
}
void Gray8_Init(void) {}
void Gray8_ReadAll(Gray8Data *data) { ++gray_read_calls; *data=sensors; }
void Key_Init(uint32_t now) { (void)now; }
uint8_t Key_StartPressed(uint32_t now) { uint8_t value=key1; (void)now; key1=0; return value; }
uint8_t Key_StopHeld(void) { return key2; }
void OLED_Init(void) { memset(screen_rows,0,sizeof(screen_rows)); oled_service_calls=0; }
void OLED_Clear(void) { memset(screen_rows,0,sizeof(screen_rows)); }
void OLED_ShowString(uint8_t x,uint8_t page,const char *value)
{
    CHECK(x==0 && page<8U); CHECK(strlen(value)<=20U);
    (void)snprintf(screen_rows[page],sizeof(screen_rows[page]),"%s",value);
}
void OLED_Service(void) { ++oled_service_calls; }
uint8_t OLED_GetBus(void) { return 1; }
void DebugUart_Init(void) {}
void DebugUart_RecordControl(uint32_t now,uint32_t interval,uint32_t cost,const EncoderData *value)
{ (void)now; (void)interval; (void)cost; (void)value; }
void DebugUart_Publish(uint32_t now,const char *state,unsigned fault,const Gray8Data *gray,
                       const EncoderData *encoder,const int16_t *pwm,const RouteState *route,
                       const OdomData *odom,uint8_t armed)
{
    (void)now; (void)gray; (void)encoder; (void)pwm; (void)odom; (void)armed;
    if (route->segment>0U && !(route->segment&1U) && route->segment<=8U &&
        route->segment!=reported_route.segment) heading_on_exit[route->segment/2U-1U]=route->heading_deg;
    (void)snprintf(reported_state,sizeof(reported_state),"%s",state);
    reported_fault=fault; reported_route=*route; stage_mask|=1U<<route->segment;
}
static void step(void) { clock_ms+=10U; App_Poll(); }
static void fixtures(void)
{
    clock_ms=0; key1=0; key2=0; motion_mode=0; reported_fault=0; stage_mask=0; saw_reverse=0; drift=0;
    gray_read_calls=0; memset(&sensors,0,sizeof(sensors)); memset(&reported_route,0,sizeof(reported_route));
    memset(heading_on_exit,0,sizeof(heading_on_exit)); memset(registers,0,sizeof(registers));
    bus_available=1; bus_fail=0; bus_data_ready=1; bus_bad_id=0; bus_bad_readback=0; bus_address=0x68;
    override_rate=0; sample_rate_override=0; sample_ax=sample_ay=0; sample_az=1;
    plant_yaw_rate=plant_yaw=0; plant_yaw_scale=1;
}
static void init_app(void) { App_Init(); App_Poll(); }
static void setup(void)
{
    fixtures(); init_app(); CHECK(strcmp(reported_state,"CAL")==0);
    for (unsigned i=0;i<IMU_CALIBRATION_SAMPLES;++i) step();
    CHECK(Imu_DataReady(Imu_GetData(),clock_ms)); CHECK(strcmp(reported_state,"IDLE")==0);
    for (unsigned i=0;i<4U;++i) CHECK(SpeedControl_GetTargets()[i]==0);
}
static void start(void) { key1=1; step(); step(); CHECK(strcmp(reported_state,"RUN")==0); }
static void stop_with_key2(void)
{
    key2=1; step(); CHECK(strcmp(reported_state,"IDLE")==0); CHECK(reported_fault==0);
    for (unsigned i=0;i<4U;++i) { CHECK(SpeedControl_GetTargets()[i]==0); CHECK(SpeedControl_GetPwm()[i]==0); }
}
static ImuData ready_at(uint32_t now,float yaw,float rate)
{
    ImuData imu={0}; imu.status=IMU_READY; imu.last_sample_ms=now; imu.yaw_deg=yaw; imu.yaw_rate_dps=rate; return imu;
}
static RouteResult update(RouteState *r,const OdomData *pose,uint32_t now,float yaw,float rate,const Gray8Data *gray,uint8_t finish)
{
    ImuData imu=ready_at(now,yaw,rate); return Route_Update(r,pose,now,finish,gray,&imu);
}
static void begin(RouteState *r,OdomData *pose)
{
    memset(pose,0,sizeof(*pose)); ImuData imu=ready_at(0,0,0); Route_Start(r,pose,0,&imu); CHECK(r->result==ROUTE_RUNNING);
}
static void test_imu(void)
{
    fixtures(); Imu_Init(); CHECK(Imu_GetData()->status==IMU_CALIBRATING);
    CHECK(registers[0x19]==4 && registers[0x1A]==3 && registers[0x1B]==8 && registers[0x1C]==0);
    for (unsigned i=0;i<50U;++i) { clock_ms+=10; Imu_Tick(clock_ms,1); }
    CHECK(Imu_GetData()->calibration_samples==50U);
    clock_ms+=10; Imu_Tick(clock_ms,0); CHECK(Imu_GetData()->calibration_samples==0U);
    for (unsigned i=0;i<IMU_CALIBRATION_SAMPLES;++i) { clock_ms+=10; Imu_Tick(clock_ms,1); }
    CHECK(Imu_DataReady(Imu_GetData(),clock_ms)); CHECK(fabsf(Imu_GetData()->bias_z_dps-gyro_bias)<0.02f);
    override_rate=1; sample_rate_override=90;
    for (unsigned i=0;i<100U;++i) { clock_ms+=10; Imu_Tick(clock_ms,0); }
    CHECK(fabsf(Imu_GetData()->yaw_deg-90)<0.1f); CHECK(fabsf(Imu_GetData()->yaw_rate_dps-90)<0.1f);
    Imu_ResetYaw(); CHECK(Imu_GetData()->yaw_deg==0);
    bus_data_ready=0; clock_ms+=IMU_MAX_SAMPLE_AGE_MS+1U; Imu_Tick(clock_ms,0);
    CHECK(Imu_GetData()->status==IMU_ERROR); CHECK(!Imu_DataReady(Imu_GetData(),clock_ms));
    fixtures(); bus_address=0x69; Imu_Init(); CHECK(Imu_GetData()->address==0x69U);
    fixtures(); bus_bad_id=1; Imu_Init(); CHECK(Imu_GetData()->status==IMU_OFFLINE);
    fixtures(); bus_bad_readback=1; Imu_Init(); CHECK(Imu_GetData()->status==IMU_ERROR);
    fixtures(); Imu_Init(); override_rate=1;
    for (unsigned i=0;i<IMU_CALIBRATION_SAMPLES;++i) { sample_rate_override=(i&1U)?3:-3; clock_ms+=10; Imu_Tick(clock_ms,1); }
    CHECK(Imu_GetData()->status==IMU_CALIBRATING); CHECK(Imu_GetData()->calibration_samples==0U);
    fixtures(); clock_ms=UINT32_MAX-1500U; Imu_Init();
    for (unsigned i=0;i<IMU_CALIBRATION_SAMPLES;++i) { clock_ms+=10; Imu_Tick(clock_ms,1); }
    CHECK(Imu_DataReady(Imu_GetData(),clock_ms));
    override_rate=1; sample_rate_override=20; clock_ms+=10; Imu_Tick(clock_ms,0);
    CHECK(fabsf(Imu_GetData()->yaw_deg-0.2f)<0.02f);
}
static void test_straight_heading_and_boundary(void)
{
    RouteState r; OdomData pose; Gray8Data line={0}; begin(&r,&pose);
    update(&r,&pose,10,0,0,NULL,0); CHECK(near(r.command_left_mm_s,402) && near(r.command_right_mm_s,398));
    update(&r,&pose,20,-5,0,NULL,0); CHECK(near(r.command_left_mm_s,442) && near(r.command_right_mm_s,358));
    update(&r,&pose,30,5,0,NULL,0); CHECK(near(r.command_left_mm_s,362) && near(r.command_right_mm_s,438));
    update(&r,&pose,40,0,20,NULL,0); CHECK(r.command_left_mm_s<r.command_right_mm_s);
    line.line_valid=1; line.active_count=1; line.position=-3.5f;
    for (unsigned i=0;i<10U;++i) update(&r,&pose,50U+i*10U,0,0,&line,0);
    CHECK(near(r.boundary_offset_deg,6)); CHECK(r.command_left_mm_s>r.command_right_mm_s);
    CHECK(near((r.command_left_mm_s+r.command_right_mm_s)*0.5f,400));
    line.marker=1;
    for (unsigned i=0;i<10U;++i) update(&r,&pose,150U+i*10U,0,0,&line,0);
    CHECK(fabsf(r.boundary_offset_deg)<0.01f); CHECK(near(r.command_left_mm_s,402));
    line.marker=0; line.position=3.5f;
    for (unsigned i=0;i<10U;++i) update(&r,&pose,250U+i*10U,0,0,&line,0);
    CHECK(near(r.boundary_offset_deg,-6)); CHECK(r.command_left_mm_s<r.command_right_mm_s);
    line.line_valid=0;
    for (unsigned i=0;i<10U;++i) update(&r,&pose,350U+i*10U,0,0,&line,0);
    CHECK(fabsf(r.boundary_offset_deg)<0.01f);
    pose.left_mm=ROUTE_FIRST_MM+20; pose.right_mm=ROUTE_FIRST_MM-24.2f;
    update(&r,&pose,500,0,0,NULL,0); CHECK(r.segment==0U);
    CHECK(near(r.command_left_mm_s,201) && near(r.command_right_mm_s,199));
    pose.right_mm+=1.2f; update(&r,&pose,510,0,0,NULL,0); CHECK(r.segment==1U);
    CHECK(near(r.heading_target_deg,90)); CHECK(r.boundary_offset_deg==0);
}
static void test_arc_angle_and_gating(void)
{
    RouteState r; OdomData pose; begin(&r,&pose); Gray8Data line={0}; line.line_valid=1; line.position=-3.5f; line.marker=1;
    uint32_t now=10;
    for (unsigned stage=0;stage<9U;++stage) {
        CHECK(r.segment==stage); CHECK(Route_InArc(&r)==(stage&1U));
        const float target=(float)((stage+1U)/2U)*90;
        CHECK(near(r.heading_target_deg,target));
        if (stage&1U) {
            const float previous=target-90;
            RouteState a=r,b=r;
            update(&a,&pose,now,previous,0,NULL,0); update(&b,&pose,now,previous,0,&line,1);
            CHECK(fabsf(a.command_left_mm_s-b.command_left_mm_s)<0.001f);
            CHECK(fabsf(a.command_right_mm_s-b.command_right_mm_s)<0.001f);
            CHECK(near((a.command_left_mm_s+a.command_right_mm_s)*0.5f,244));
            CHECK(a.command_left_mm_s<=600 && a.command_right_mm_s>=-600);
            pose.left_mm=r.start_left_mm+r.goal_left_mm; pose.right_mm=r.start_right_mm+r.goal_right_mm;
            update(&r,&pose,now+10,target-30,0,NULL,0); CHECK(r.segment==stage); /* encoder goal alone cannot finish */
            update(&r,&pose,now+20,target,40,NULL,0); CHECK(r.segment==stage); /* brake yaw inertia */
            CHECK(r.command_left_mm_s<r.command_right_mm_s);
            update(&r,&pose,now+30,target+5,5,&line,1); CHECK(r.segment==stage);
            CHECK(r.command_left_mm_s<r.command_right_mm_s); /* correct overshoot */
            update(&r,&pose,now+40,target,0,NULL,0); CHECK(r.segment==stage+1U);
            CHECK(r.command_left_mm_s>0 && r.command_right_mm_s>0); /* no fixed wait tick */
        } else {
            CHECK(near((r.goal_left_mm+r.goal_right_mm)*0.5f,stage==0U?944:(stage==8U?450:1300)));
            if (stage==8U) break;
            pose.left_mm=r.start_left_mm+r.goal_left_mm; pose.right_mm=r.start_right_mm+r.goal_right_mm;
            update(&r,&pose,now,target,0,NULL,0); CHECK(r.segment==stage+1U);
        }
        now+=100;
    }
    CHECK(Route_FinishArmed(&r,&pose));
    CHECK(update(&r,&pose,1000,360,0,NULL,1)==ROUTE_COMPLETE);
    CHECK(r.command_left_mm_s==0 && r.command_right_mm_s==0);
    begin(&r,&pose); ImuData stale=ready_at(0,0,0);
    CHECK(Route_Update(&r,&pose,31,0,NULL,&stale)==ROUTE_IMU_ERROR);
    CHECK(r.command_left_mm_s==0 && r.command_right_mm_s==0);
}
static void test_app_lap(uint8_t final_marker,uint8_t marker_stuck,float yaw_scale)
{
    setup(); start(); plant_yaw_scale=yaw_scale;
    for (unsigned i=0;i<10000U && strcmp(reported_state,"RUN")==0;++i) {
        sensors.marker=marker_stuck || clock_ms<3400U ||
            (reported_route.segment==2U && reported_route.progress_left_mm<200) ||
            (final_marker && reported_route.segment==8U &&
             (reported_route.progress_left_mm+reported_route.progress_right_mm)*0.5f>=140);
        step();
    }
    if (final_marker && !marker_stuck) {
        CHECK(strcmp(reported_state,"BRAKE")==0);
        CHECK((reported_route.progress_left_mm+reported_route.progress_right_mm)*0.5f<151);
        for (unsigned i=0;i<200U && strcmp(reported_state,"BRAKE")==0;++i) step();
        CHECK(strcmp(reported_state,"DONE")==0); CHECK(reported_fault==0);
    } else { CHECK(strcmp(reported_state,"FAULT")==0); CHECK(reported_fault==10U); }
    CHECK(stage_mask==0x1FFU);
    for (unsigned i=0;i<4U;++i) {
        CHECK(fabsf(heading_on_exit[i]-(float)(i+1U)*90)<2.1f);
        CHECK(SpeedControl_GetTargets()[i]==0);
    }
}
static void test_app_imu_faults_and_manual_stop(void)
{
    fixtures(); init_app(); key1=1; step(); CHECK(reported_fault==12U);
    for (unsigned i=0;i<4U;++i) CHECK(SpeedControl_GetTargets()[i]==0);
    fixtures(); bus_available=0; init_app(); key1=1; step(); CHECK(reported_fault==12U);
    setup(); start(); bus_fail=1; step(); CHECK(reported_fault==13U);
    for (unsigned i=0;i<4U;++i) CHECK(SpeedControl_GetTargets()[i]==0);
    setup(); start(); bus_data_ready=0;
    for (unsigned i=0;i<4U;++i) step();
    CHECK(reported_fault==13U);
    setup(); start(); key1=1; stop_with_key2();
    setup(); start(); clock_ms+=60U; App_Poll(); CHECK(reported_fault==5U);
}
static void test_app_drift_and_gray_gate(void)
{
    setup(); start(); drift=1;
    for (unsigned i=0;i<150U;++i) step();
    CHECK(reported_route.segment==0U); CHECK(fabsf(Imu_GetData()->yaw_deg)<3);
    CHECK(SpeedControl_GetTargets()[0]-SpeedControl_GetTargets()[2]>4);
    stop_with_key2();
    setup(); start();
    for (unsigned i=0;i<2000U && reported_route.segment==0U;++i) step();
    CHECK(reported_route.segment==1U);
    const unsigned calls=gray_read_calls;
    sensors.line_valid=1; sensors.marker=1; sensors.position=-3.5f;
    for (unsigned i=0;i<2000U && reported_route.segment==1U;++i) {
        step(); CHECK(gray_read_calls==calls); CHECK(reported_fault==0);
    }
    CHECK(reported_route.segment==2U); CHECK(reported_route.boundary_offset_deg==0);
    step(); CHECK(gray_read_calls==calls+1U); CHECK(reported_route.boundary_offset_deg==0); /* marker bypass */
    stop_with_key2();
}
static void test_encoder_faults_disabled(void)
{
    setup(); start(); motion_mode=1;
    for (unsigned i=0;i<200U;++i) { step(); CHECK(strcmp(reported_state,"RUN")==0); CHECK(reported_fault==0); }
    stop_with_key2();
    setup(); start(); motion_mode=2;
    for (unsigned i=0;i<100U;++i) { step(); CHECK(strcmp(reported_state,"RUN")==0); CHECK(reported_fault==0); }
    stop_with_key2();
}
static void test_signed_speed(void)
{
    float values[4]={100,100,100,100};
    float measured[4]={0};
    uint8_t passed_zero=0;
    setup();
    SpeedControl_SetTargets(values);
    for(unsigned i=0;i<30;++i) SpeedControl_Tick(measured,10);
    CHECK(near(SpeedControl_GetRampedTargets()[0],100));
    for(unsigned i=0;i<4;++i) values[i]=-100;
    SpeedControl_SetTargets(values);
    for(unsigned i=0;i<30;++i){
        SpeedControl_Tick(measured,10);
        const float speed=SpeedControl_GetRampedTargets()[0];
        if(speed==0) passed_zero=1;
        if(speed<0) CHECK(passed_zero);
    }
    CHECK(near(SpeedControl_GetRampedTargets()[0],-100));
    for(unsigned i=0;i<4;++i) measured[i]=-500;
    SpeedControl_Tick(measured,10);
    for(unsigned i=0;i<4;++i) CHECK(SpeedControl_GetPwm()[i]<=0);
    for(unsigned i=0;i<4;++i) {values[i]=0; measured[i]=0;}
    SpeedControl_SetTargets(values); SpeedControl_Tick(measured,10);
    CHECK(SpeedControl_GetRampedTargets()[0]<0); /* decelerate instead of an abrupt zero shortcut */
    for(unsigned i=0;i<20;++i) SpeedControl_Tick(measured,10);
    for(unsigned i=0;i<4;++i) CHECK(SpeedControl_GetRampedTargets()[i]==0 && SpeedControl_GetPwm()[i]==0);
    for(unsigned i=0;i<4;++i) {values[i]=100; measured[i]=500;}
    SpeedControl_SetTargets(values);
    for(unsigned i=0;i<30;++i) SpeedControl_Tick(measured,10);
    for(unsigned i=0;i<4;++i) CHECK(SpeedControl_GetPwm()[i]>=0);
}
static void test_hmi_timer_and_rows(void)
{
    HMI_Init();
    CHECK(strcmp(screen_rows[0],"S:IDLE")==0);
    CHECK(strcmp(screen_rows[2],"T:00:00.00")==0);
    HMI_Start(1000U);
    HMI_Tick(1250U,"RUN",0U,123.9f,200.9f);
    CHECK(strncmp(screen_rows[2],"T:00:00.25",10)==0);
    CHECK(strncmp(screen_rows[0],"S:RUN",5)==0);
    HMI_Tick(1350U,"RUN",0U,-17.9f,200.9f);
    CHECK(strncmp(screen_rows[4],"D:-17mm",7)==0);
    HMI_Tick(1450U,"RUN",9U,-17.9f,200.9f);
    CHECK(strncmp(screen_rows[6],"F:9",3)==0);
    HMI_Stop(1520U);
    HMI_Tick(1550U,"IDLE",0U,0,0);
    CHECK(strncmp(screen_rows[2],"T:00:00.52",10)==0);
    HMI_Tick(1850U,"IDLE",0U,0,0);
    CHECK(strncmp(screen_rows[2],"T:00:00.52",10)==0);
    HMI_Start(2000U); HMI_Tick(2100U,"RUN",0U,0,20);
    CHECK(strncmp(screen_rows[2],"T:00:00.10",10)==0);
    HMI_Init(); HMI_Tick(100U,"IDLE",0U,0,0);
    CHECK(strncmp(screen_rows[2],"T:00:00.00",10)==0);
    HMI_Start(UINT32_MAX-49U); HMI_Tick(UINT32_MAX-1U,"RUN",0U,0,0);
    HMI_Stop(100U); HMI_Tick(200U,"IDLE",0U,0,0);
    CHECK(strncmp(screen_rows[2],"T:00:00.15",10)==0);
    CHECK(oled_service_calls>=3U);
}
static void test_hmi_app_stop(void)
{
    setup(); start();
    for(unsigned i=0;i<50U;++i) step();
    CHECK(strncmp(screen_rows[0],"S:RUN",5)==0);
    stop_with_key2();
    for(unsigned i=0;i<12U;++i) step();
    char frozen[21]; memcpy(frozen,screen_rows[2],sizeof(frozen));
    for(unsigned i=0;i<40U;++i) step();
    CHECK(strcmp(frozen,screen_rows[2])==0);
    CHECK(strncmp(screen_rows[0],"S:IDLE",6)==0);
}
int main(void)
{
    test_imu(); test_straight_heading_and_boundary(); test_arc_angle_and_gating();
    test_app_lap(1,0,0.85f); test_app_lap(0,0,1.15f); test_app_lap(1,1,1.0f);
    test_app_imu_faults_and_manual_stop(); test_app_drift_and_gray_gate(); test_encoder_faults_disabled();
    test_signed_speed(); test_hmi_timer_and_rows(); test_hmi_app_stop();
    printf("PASS: %u assertions; V10 real MPU calibration/integration, gyro heading/90-degree arcs, arc gray suppression, encoder speed/distance, final bar, HMI and manual stop\n",checks);
    return 0;
}
