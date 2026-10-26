#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
void Motor_Init(void);
/* Logical PWM excludes the configured dead-zone compensation. */
void Motor_SetPwm(unsigned id, int16_t logical_pwm);
void Motor_StopAll(uint8_t brake);
/* Last commanded timer compare pairs: M1 A/B, M2 A/B, M3 A/B, M4 A/B.
 * These are output commands, not a measurement of motor voltage/current. */
const uint16_t *Motor_GetOutputs(void);
#endif
