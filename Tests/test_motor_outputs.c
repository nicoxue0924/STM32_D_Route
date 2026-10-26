#include "motor.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
TIM_TypeDef fake_tim1,fake_tim8;
TIM_HandleTypeDef htim1,htim8;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
void Error_Handler(void) { abort(); }
int main(void)
{
    const uint16_t *out=Motor_GetOutputs();
    Motor_Init();
    for (unsigned i=0;i<8U;++i) assert(out[i]==0);
    Motor_SetPwm(0,10); assert(fake_tim8.CCR1==0 && fake_tim8.CCR2==2010);
    assert(out[0]==0 && out[1]==2010);
    Motor_SetPwm(1,-20); assert(fake_tim8.CCR3==2020 && fake_tim8.CCR4==0);
    assert(out[2]==2020 && out[3]==0);
    Motor_SetPwm(2,300); assert(fake_tim1.CCR1==2300 && fake_tim1.CCR2==0);
    Motor_SetPwm(3,32767); assert(fake_tim1.CCR3==2800 && fake_tim1.CCR4==0);
    assert(out[6]==2800 && out[7]==0);
    Motor_SetPwm(99,100); assert(out[6]==2800);
    Motor_SetPwm(0,0); assert(out[0]==0 && out[1]==0);
    Motor_StopAll(1);
    for (unsigned i=0;i<8U;++i) assert(out[i]==3600);
    Motor_StopAll(0);
    for (unsigned i=0;i<8U;++i) assert(out[i]==0);
    puts("PASS: logged bridge commands match timer writes, signs, dead zone, clamp and braking");
    return 0;
}
