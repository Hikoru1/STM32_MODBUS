#ifndef __PETITMODBUSPORT__H
#define __PETITMODBUSPORT__H

#include "PetitModbus.h"

#define USART1_DR_BASE  0x40013804
#define USART2_DR_BASE  0x40004404
#define USART3_DR_BASE	0x40004804
#define USART6_DR_BASE	0x40011400

extern unsigned char    U3TxDmaBuf[PETITMODBUS_TRANSMIT_BUFFER_SIZE];                                          // DMA Transmit Isleminde Kullanilacaktir!

extern unsigned char    PetitReceiveBuffer[PETITMODBUS_RECEIVE_BUFFER_SIZE];
extern unsigned char    PetitReceiveCounter;

extern void             PetitModBus_UART_Initialise(void);
extern void             PetitModBus_TIMER_Initialise(void);
extern void             PetitModBus_UART_Putch(unsigned char *s,unsigned int size);
extern unsigned char    PetitModBus_UART_String(unsigned char *s, unsigned int Length);

extern void             PetitModBus_TimerValues(void);
                  // DMA Transmit Isleminde Kullanilacaktir!
#endif
