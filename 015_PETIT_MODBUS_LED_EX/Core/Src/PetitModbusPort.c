#include "PetitModbus.h"
#include "PetitModbusPort.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"
unsigned char   PetitReceiveBuffer[PETITMODBUS_RECEIVE_BUFFER_SIZE];
unsigned char   PetitReceiveCounter=0;

//unsigned char   U3TxDmaBuf[PETITMODBUS_TRANSMIT_BUFFER_SIZE]; 
void PetitModBus_UART_Initialise(void)
{
    General_USART_Init();
}

void PetitModBus_TIMER_Initialise(void)
{
    General_Timer_Init();
}

void PetitModBus_UART_Putch(unsigned char *s,unsigned int size)
{
//	//Wait for the uart to finish sending the byte.
//	while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
//	
//	USART_SendData(USART6,c);
	int i = 0;
	for(i = 0;i < size;i++ )
	{
		while( !(USART6->SR & 0x00000040) );
		USART_SendData(USART6, *(s+i));
		//*s++;
	}
}

void ResetAndDisableReceive(void)
{
    PetitReceiveCounter = 0;
}

unsigned char PetitModBus_UART_String(unsigned char *s, unsigned int Length)
{
	DMA_InitTypeDef   DMA_InitStructure;
//	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_DeInit(DMA2_Stream6);																										// USART1 TX icin Channel1 secilmeli
	DMA_Cmd(DMA2_Stream6, DISABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr 		= (uint32_t)(&USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr  			= (uint32_t)&(*s);
	DMA_InitStructure.DMA_DIR 									= DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize 						= Length;
	DMA_InitStructure.DMA_PeripheralInc 				= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc    					= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize 				= DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc 						= DMA_MemoryInc_Enable;  
	DMA_InitStructure.DMA_Mode 									= DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority 							= DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode 							= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 				= DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_FIFOThreshold 				= DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_MemoryBurst 					= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
	
	/* Configure TX DMA */   
	//  DMA_InitStructure.DMA_BufferSize = USART_TX_BUFFSIZE ;   
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;     
	
	
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

	/* Enable DMA1 Channel7 */
	DMA_Cmd(DMA2_Stream6, ENABLE);
	
	/* Enable USART1 DMA TX request */
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	
	
  ResetAndDisableReceive();

	toggle();
	
	return TRUE;
}

/***************************Interrupt For Slave********************************/


void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
	{
		toggle_ORANGE();
		USART_ClearFlag(USART6, USART_IT_RXNE);
		
		PetitReceiveBuffer[PetitReceiveCounter]   =USART_ReceiveData(USART6);
	//		USART_SendData(USART6,&PetitReceiveBuffer[PetitReceiveCounter]);
    PetitReceiveCounter++;

    if(PetitReceiveCounter>PETITMODBUS_RECEIVE_BUFFER_SIZE)  
        PetitReceiveCounter=0;

    PetitModbusTimerValue=0;
	}
}

/******************************************************************************/
void PetitModBus_TimerValues(void)
{
    PetitModbusTimerValue++;
}

//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
//	{
//		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
//		
//		PetitModBus_TimerValues();
//	}
//}
