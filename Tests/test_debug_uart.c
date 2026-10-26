#include "debug_uart.h"
#include "imu.h"
#include "usart.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
static ImuData imu;
static uint32_t clock_ms;
static char received[2048];
static const uint8_t *in_flight;
static size_t in_flight_length;
static HAL_StatusTypeDef send_result;
static float targets[4]={402,402,398,398};
static float ramped[4]={100.5f,100.5f,100.5f,100.5f};
static uint16_t outputs[8]={0,2010,0,2020,2030,0,2040,0};
static EncoderData encoder;
static Gray8Data gray;
static RouteState route;
static OdomData odom;
static const int16_t pwm[4]={10,20,30,40};

uint32_t HAL_GetTick(void) { return clock_ms; }
const ImuData *Imu_GetData(void) { return &imu; }
const char *Route_SegmentName(const RouteState *value) { (void)value; return "S1"; }
const float *SpeedControl_GetTargets(void) { return targets; }
const float *SpeedControl_GetRampedTargets(void) { return ramped; }
const uint16_t *Motor_GetOutputs(void) { return outputs; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t length)
{
    if (send_result!=HAL_OK) return send_result;
    assert(length<sizeof(received));
    memcpy(received,data,length); received[length]=0;
    in_flight=data; in_flight_length=length;
    uart->gState=HAL_UART_STATE_BUSY_TX;
    return HAL_OK;
}
static void publish(uint32_t now)
{
    clock_ms=now;
    DebugUart_Publish(now,"IDLE",0,&gray,&encoder,pwm,&route,&odom,0);
}
static void ready(void) { huart1.gState=HAL_UART_STATE_READY; }
static void contains(const char *value)
{
    if (!strstr(received,value)) {
        fprintf(stderr,"Missing [%s] in:\n%s",value,received);
        assert(0);
    }
}
static void fixture(uint32_t now)
{
    memset(&imu,0,sizeof(imu)); memset(&encoder,0,sizeof(encoder));
    memset(&gray,0,sizeof(gray)); memset(&route,0,sizeof(route)); memset(&odom,0,sizeof(odom));
    huart1.Init.BaudRate=115200; ready(); send_result=HAL_OK; clock_ms=now;
    imu.status=IMU_READY; imu.bias_z_dps=1.25f; imu.last_sample_ms=now;
    encoder.count[0]=123; encoder.delta[0]=8; encoder.speed_mm_s[0]=100.25f;
    gray.raw[0]=1; gray.active_mask=128; gray.active_count=1;
    gray.line_valid=1; gray.position=3.5f;
    route.command_left_mm_s=402; route.command_right_mm_s=398;
    DebugUart_Init();
}
static void test_complete_diagnostics_and_busy_buffer(void)
{
    char pending[2048];
    fixture(0); publish(0);
    contains("log=V10L1 type=cfg"); contains("baud=115200");
    contains(" crc=");
    contains("motor_sign=-1,-1,1,1"); contains("enc_sign=1,1,-1,-1");
    ready(); publish(100);
    contains("log=V10L1 type=data"); contains("cnt=123,0,0,0");
    contains("v10=1003,0,0,0"); contains("target10=4020,4020,3980,3980");
    contains("ramp10=1005,1005,1005,1005"); contains("denc=8,0,0,0");
    contains("out=0,2010,0,2020,2030,0,2040,0"); contains("bias1000=1250");
    memcpy(pending,in_flight,in_flight_length);
    publish(200);
    assert(memcmp(pending,in_flight,in_flight_length)==0);
    ready(); publish(300); contains("drop=1");
    ready(); publish(5000); contains("type=cfg");
}
static void test_transmit_failure_is_reported(void)
{
    fixture(0); send_result=HAL_ERROR; publish(0);
    send_result=HAL_OK; publish(100); contains("type=cfg");
    ready(); publish(200); contains("txerr=1"); contains("drop=1");
}
static void test_wraparound_and_finite_bounded_records(void)
{
    fixture(UINT32_MAX-50U); publish(UINT32_MAX-50U); ready();
    encoder.count[0]=INT32_MIN; encoder.count[1]=INT32_MAX;
    encoder.delta[0]=INT16_MIN; encoder.delta[1]=INT16_MAX;
    encoder.speed_mm_s[0]=NAN; encoder.speed_mm_s[1]=INFINITY;
    imu.yaw_deg=INFINITY; imu.yaw_rate_dps=-INFINITY;
    publish(49U);
    contains("type=data"); contains("cnt=-2147483648,2147483647,0,0");
    contains("badnum=");
    assert(strlen(received)<=1024U);
    assert(strstr(received,"\r\n")!=NULL);
    assert(strstr(received,"nan")==NULL && strstr(received,"inf")==NULL);
}
static void test_control_window_extrema_and_reset(void)
{
    fixture(0); publish(0); ready();
    DebugUart_RecordControl(10,10,4,&encoder);
    encoder.speed_mm_s[0]=80.5f;
    DebugUart_RecordControl(40,30,2,&encoder);
    publish(100);
    contains("vmin10=805,0,0,0"); contains("vmax10=1003,0,0,0");
    contains("dt=30 dtmin=10 dtmax=30 costmax=4 ctrl_n=2 ctrl_age=60");
    ready(); encoder.speed_mm_s[0]=120.5f;
    DebugUart_RecordControl(200,10,1,&encoder); publish(200);
    contains("vmin10=1205,0,0,0"); contains("dt=10 dtmin=10 dtmax=10 costmax=1 ctrl_n=1");
}
static void test_oversized_record_is_reported_without_losing_window(void)
{
    char long_state[1600];
    memset(long_state,'X',sizeof(long_state)-1); long_state[sizeof(long_state)-1]=0;
    fixture(0); publish(0); ready();
    DebugUart_RecordControl(10,10,4,&encoder);
    DebugUart_Publish(100,long_state,0,&gray,&encoder,pwm,&route,&odom,0);
    contains("type=overflow");
    ready(); DebugUart_RecordControl(200,10,2,&encoder); publish(200);
    contains("ctrl_n=2"); contains("drop=1");
}
static void test_negative_saturation_is_flagged(void)
{
    fixture(0); publish(0); ready(); imu.yaw_deg=-3.0e20f; publish(100);
    contains("yaw100=-2147483648"); contains("badnum=2");
}
static void test_transient_invalid_speed_is_not_hidden(void)
{
    fixture(0); publish(0); ready();
    DebugUart_RecordControl(10,10,1,&encoder);
    encoder.speed_mm_s[0]=NAN; DebugUart_RecordControl(20,10,1,&encoder);
    encoder.speed_mm_s[0]=101.5f; DebugUart_RecordControl(30,10,1,&encoder);
    publish(100); contains("badnum=1");
    contains("vmin10=1003,0,0,0"); contains("vmax10=1015,0,0,0");
    ready(); encoder.speed_mm_s[0]=NAN; DebugUart_RecordControl(110,10,1,&encoder);
    encoder.speed_mm_s[0]=120.5f; DebugUart_RecordControl(120,10,1,&encoder);
    publish(200); contains("badnum=2"); contains("vmin10=1205,0,0,0");
}
int main(int argc,char **argv)
{
    if (argc==2 && strcmp(argv[1],"--emit")==0) {
        fixture(0); publish(0); fputs(received,stdout);
        ready(); DebugUart_RecordControl(100,10,4,&encoder); publish(100); fputs(received,stdout);
        return 0;
    }
    test_complete_diagnostics_and_busy_buffer();
    test_transmit_failure_is_reported();
    test_wraparound_and_finite_bounded_records();
    test_control_window_extrema_and_reset();
    test_oversized_record_is_reported_without_losing_window();
    test_negative_saturation_is_flagged();
    test_transient_invalid_speed_is_not_hidden();
    puts("PASS: diagnostic fields, busy-buffer ownership, failures, wraparound, numeric bounds");
    return 0;
}
