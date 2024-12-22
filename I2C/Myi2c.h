#ifndef __Myi2c_H__
#define __Myi2c_H__

//****************************************
// 定义 I2C 通信的相关引脚
//****************************************
#define i2c_SCL GPIO_Pin_6
#define i2c_SDA GPIO_Pin_7
#define i2c_PORT GPIOA

#define Delaytime 25
#define mdelay Delay_ms

#define Soft_I2C_READY 0x00
#define Soft_I2C_BUS_BUSY 0x01
#define Soft_I2C_BUS_ERROR 0x02

void i2c_Init(void);
uint8_t i2c_Start(void);
void i2c_Stop(void);
void i2c_SendACK(void);
void i2c_SendNACK(void);

uint8_t i2c_SendByte(uint8_t Byte);

uint8_t i2c_ReceiveByte_NACK(void);
uint8_t i2c_ReceiveByte_ACK(void);


uint8_t i2c_WriteData(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
uint8_t i2c_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *read_databuf);
int i2c_WriteRegister(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t *data_ptr);
int i2c_ReadRegister(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t *data_ptr);

#endif // MYIC_H
