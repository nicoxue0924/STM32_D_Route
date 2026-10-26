#ifndef DEBUG_UART_H
#define DEBUG_UART_H
#include <stdint.h>
#include "encoder.h"
#include "gray8.h"
#include "route.h"
void DebugUart_Init(void);
/* Observe only: called after each control tick, including early fault returns. */
void DebugUart_RecordControl(uint32_t now_ms, uint32_t interval_ms,
                             uint32_t cost_ms, const EncoderData *encoder);
void DebugUart_Publish(uint32_t now_ms, const char *state, unsigned fault,
                       const Gray8Data *gray, const EncoderData *encoder,
                       const int16_t *pwm, const RouteState *route,
                       const OdomData *odom, uint8_t finish_armed);
#endif
