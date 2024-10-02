#ifndef INC_MODBUS_LIB_H_
#define INC_MODBUS_LIB_H_


/*
 ***************************************************
CRC HESAPLAMA FONKSİYONU
 ***************************************************
*/
void Modbus_Process(void);
uint16_t READ_FUNC(uint8_t rx_message[100]);
uint16_t PRESET_MUL_REG_FUNC(uint8_t rx_message[100]);
uint16_t WRITE_SINGLE_REG(uint8_t rx_message[100]);


#endif /* INC_MODBUS_LIB_H_ */
