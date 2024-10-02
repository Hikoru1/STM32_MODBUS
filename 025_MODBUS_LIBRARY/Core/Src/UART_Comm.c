/*
 * UART_Comm.c
 *
 *  Created on: Sep 23, 2024
 *      Author: koruc
 */


#include "UART_Comm.h"

void UART_Send( UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
    HAL_UART_Transmit(huart, data, size, 1000);
}

void UART_Receive( UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
    HAL_UART_Receive(huart, data, size, 1000);
}
