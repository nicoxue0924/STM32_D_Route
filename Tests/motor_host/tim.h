#ifndef HOST_MOTOR_TIM_H
#define HOST_MOTOR_TIM_H
#include "usart.h"
typedef struct { uint32_t CCR1,CCR2,CCR3,CCR4; } TIM_TypeDef;
typedef struct { unsigned unused; } TIM_HandleTypeDef;
extern TIM_TypeDef fake_tim1,fake_tim8;
extern TIM_HandleTypeDef htim1,htim8;
#define TIM1 (&fake_tim1)
#define TIM8 (&fake_tim8)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer,uint32_t channel);
void Error_Handler(void);
#endif
