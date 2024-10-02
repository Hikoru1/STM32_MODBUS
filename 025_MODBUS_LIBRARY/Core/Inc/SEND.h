/*
 * SEND.h
 *
 *  Created on: Sep 19, 2024
 *      Author: koruc
 */

#ifndef INC_SEND_H_
#define INC_SEND_H_

#include "MODBUS_LIB.h"

	  HAL_UART_Receive(&huart2, rxbuffer, sizeof(rxbuffer),1000);

	  if(rxbuffer[0]==SLAVEID && rxbuffer[1]==0x03){
		  READ_FUNC(rxbuffer);
		  HAL_UART_Transmit(&huart2, response, response_size, 1000);


	  }
	  else if(rxbuffer[0]==SLAVEID && rxbuffer[1]==0x10){
		  PRESET_MUL_REG_FUNC(rxbuffer);
	  }

	  else if((rxbuffer[0]==SLAVEID) && (rxbuffer[1]==0x06)){
		  WRITE_SINGLE_REG(rxbuffer);
	  }

  }


#endif /* INC_SEND_H_ */
