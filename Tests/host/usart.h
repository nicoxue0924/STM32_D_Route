#ifndef HOST_USART_H
#define HOST_USART_H
#include "main.h"
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct {
    unsigned gState;
    struct { uint32_t BaudRate; } Init;
} UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
extern UART_HandleTypeDef huart1;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size);
#endif
