/*
 * MODBUS_LIB.c
 *
 *  Created on: Sep 23, 2024
 *      Author: koruc
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <REGISTERS.h>
#include "UART_Comm.h"


uint8_t txbuffer[100];
uint8_t rxbuffer[100];
uint16_t registers[REGISTERS];


uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*
 ***************************************************
0x03 READ MULTIPLE REGISTER FONKSİYONU
 ***************************************************
*/
uint16_t READ_FUNC(uint8_t rx_message[100]){	//TUM REGISTERLARIN OKUNMASI
	 uint16_t firs_Reg = rx_message[2]<<8 |rx_message[3];
	 uint16_t num_Reg = rx_message[4]<<8 | rx_message[5];

	 uint8_t response[5+2*num_Reg+2];
	 response[0]=SLAVEID;
	 response[1]=0x03;
	 response[2]= 2*num_Reg;
	 for(int i=0; i<num_Reg;i++){
		 response[3+i*2] = (registers[firs_Reg+i])>>8 & 0xFF ;
		 response[4+i*2] = registers[firs_Reg+i] & 0xFF ;

	 }
 	uint16_t crc2 = Modbus_CRC16(response, 3 + 2 * num_Reg);
 	response[3 + 2 * num_Reg] = crc2 & 0xFF; // CRC'nin düşük baytı
 	response[4 + 2 * num_Reg] = (crc2 >> 8) & 0xFF; // CRC'nin yüksek baytı

 	// Cevabı gönder
 	int response_size = 3 + 2 * num_Reg + 2;
 	UART_Send(&huart2, response, response_size); // Cevabı gönder

}

/*
 ***************************************************
0x10 WRITE MULTIPLE REGISTER FONKSİYONU
 ***************************************************
*/

uint16_t PRESET_MUL_REG_FUNC(uint8_t rx_message[100]){

	uint16_t start_Address = (rx_message[2] << 8) | rx_message[3];
	uint16_t numberof_registers = (rx_message[4] << 8) | rx_message[5];
	uint8_t numof_databytes = rx_message[6];

	// Girilen verileri registerlara yazmak için
	for (int i = 0; i < numberof_registers; i++)
	{
	    registers[start_Address + i] = (rx_message[7 + i*2] << 8) | rx_message[8 + i*2];

	     }

	     // Cevap mesajı hazırlama
	     txbuffer[0] = SLAVEID;
	     txbuffer[1] = 0x10;
	     txbuffer[2] = rx_message[2];
	     txbuffer[3] = rx_message[3];
	     txbuffer[4] = rx_message[4];
	     txbuffer[5] = rx_message[5];

	     uint16_t crc = Modbus_CRC16(txbuffer, 6);
	     txbuffer[6] = crc & 0xFF;
	     txbuffer[7] = (crc >> 8) & 0xFF;
	     UART_Send(&huart2, txbuffer, 8); // Cevabı gönder

}

/*
 ***************************************************
0x06 WRITE SINGLE REGISTER FONKSİYONU
 ***************************************************
*/

uint16_t WRITE_SINGLE_REG(uint8_t rx_message[100]){
	uint16_t address = (rxbuffer[2]<<8) | rxbuffer[3];
	uint16_t value = ((rxbuffer[4]<<8) | rxbuffer[5]);

	registers[address] = value;										// Register'a yazma

	uint8_t response[8];
	for (int i = 0; i < 6; i++) {								// Cevabı hazırlama
		response[i] = rxbuffer[i];
	}

	uint16_t crc = Modbus_CRC16(response, 6);					// CRC hesaplama
	response[6] = crc & 0xFF;
	response[7] = (crc >> 8) & 0xFF;

	// Cevabı gönder
    UART_Send(&huart2, response, sizeof(response)); // Cevabı gönder

}



void Modbus_Process(){


		 if(rxbuffer[0]==SLAVEID && rxbuffer[1]==0x03){
			  READ_FUNC(rxbuffer);

		  }
		  else if(rxbuffer[0]==SLAVEID && rxbuffer[1]==0x10){
			  PRESET_MUL_REG_FUNC(rxbuffer);
		  }

		  else if((rxbuffer[0]==SLAVEID) && (rxbuffer[1]==0x06)){
			  WRITE_SINGLE_REG(rxbuffer);
		  }
		    UART_Receive(&huart2, rxbuffer, sizeof(rxbuffer)); // Veriyi al

}






