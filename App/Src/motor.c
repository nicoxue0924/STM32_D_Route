/* The sign/dead-zone convention follows the supplied Yahboom motor example. */
#include "motor.h"
#include "tim.h"
#include "board_config.h"
static const int8_t direction[MOTOR_COUNT] = MOTOR_DIRECTION_SIGNS;
static uint16_t outputs[MOTOR_COUNT*2U];

static void write_pair(unsigned id, uint32_t a, uint32_t b)
{
    /* Clear both old values first when switching direction. */
    switch (id) {
    case 0: TIM8->CCR1=0; TIM8->CCR2=0; TIM8->CCR1=a; TIM8->CCR2=b; break;
    case 1: TIM8->CCR3=0; TIM8->CCR4=0; TIM8->CCR3=a; TIM8->CCR4=b; break;
    case 2: TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR1=a; TIM1->CCR2=b; break;
    case 3: TIM1->CCR3=0; TIM1->CCR4=0; TIM1->CCR3=a; TIM1->CCR4=b; break;
    default: return;
    }
    outputs[id*2U]=(uint16_t)a;
    outputs[id*2U+1U]=(uint16_t)b;
}

void Motor_Init(void)
{
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    Motor_StopAll(0);
    for (unsigned i=0; i<4; ++i) {
        if (HAL_TIM_PWM_Start(&htim1, channels[i]) != HAL_OK) Error_Handler();
        if (HAL_TIM_PWM_Start(&htim8, channels[i]) != HAL_OK) Error_Handler();
    }
}

void Motor_SetPwm(unsigned id, int16_t logical_pwm)
{
    int32_t pwm = logical_pwm;
    const int32_t limit = (int32_t)MOTOR_PWM_LIMIT_COUNTS - (int32_t)MOTOR_DEADZONE_COUNTS;
    if (id >= MOTOR_COUNT) return;
    if (pwm > limit) pwm = limit;
    if (pwm < -limit) pwm = -limit;
    if (pwm > 0) pwm += (int32_t)MOTOR_DEADZONE_COUNTS;
    else if (pwm < 0) pwm -= (int32_t)MOTOR_DEADZONE_COUNTS;
    pwm *= direction[id];
    if (pwm >= 0) write_pair(id, (uint32_t)pwm, 0);
    else write_pair(id, 0, (uint32_t)(-pwm));
}

void Motor_StopAll(uint8_t brake)
{
    uint32_t duty = brake ? MOTOR_PWM_PERIOD_COUNTS : 0U;
    for (unsigned i=0; i<MOTOR_COUNT; ++i) write_pair(i, duty, duty);
}
const uint16_t *Motor_GetOutputs(void) { return outputs; }
