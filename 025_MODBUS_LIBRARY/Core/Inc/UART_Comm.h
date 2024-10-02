/*
 * UART_Comm.h
 *
 *  Created on: Sep 23, 2024
 *      Author: koruc
 */

#ifndef INC_UART_COMM_H_
#define INC_UART_COMM_H_


#include "stm32f4xx_hal.h" // HAL kütüphanesi
extern UART_HandleTypeDef huart2;

void UART_Send( UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
void UART_Receive( UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);


#endif /* INC_UART_COMM_H_ */
