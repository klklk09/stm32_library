#include "stm32f10x.h" // Device header
#include "Delay.h"
#include "Myi2c.h"

#define mdelay Delay_ms

#define Soft_I2C_READY 0x00
#define Soft_I2C_BUS_BUSY 0x01
#define Soft_I2C_BUS_ERROR 0x02

// static void Soft_I2C_Delay(uint32_t dly)
// {
//   while (--dly)
//     ; // dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
// }

static unsigned short RETRY_IN_MLSEC = 55;

/**
 * @brief  设置iic重试时间
 * @param  ml_sec：重试的时间，单位毫秒
 * @retval 重试的时间，单位毫秒
 */
void Set_I2C_Retry(unsigned short ml_sec)
{
  RETRY_IN_MLSEC = ml_sec;
}

/**
 * @brief  获取设置的iic重试时间
 * @param  none
 * @retval none
 */
unsigned short Get_I2C_Retry(void)
{
  return RETRY_IN_MLSEC;
}

/**
 * @brief i2c初始化,配置I2C的SCL和SDA引脚
 *
 */
void i2c_Init(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2c_SCL | i2c_SDA;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(i2c_PORT, &GPIO_InitStructure);

  GPIO_SetBits(i2c_PORT, i2c_SCL | i2c_SDA); // 设置为高电平
  Set_I2C_Retry(5);                          // 设置重试时间
}

void SCL(uint8_t BitValue)
{
  GPIO_WriteBit(i2c_PORT, i2c_SCL, (BitAction)BitValue);
  Delay_us(Delaytime);
}

void SDA(uint8_t BitValue)
{
  GPIO_WriteBit(i2c_PORT, i2c_SDA, (BitAction)BitValue);
  Delay_us(Delaytime);
}

uint8_t SDA_ReadState(void)
{
  uint8_t ReadValue;
  ReadValue = GPIO_ReadInputDataBit(i2c_PORT, i2c_SDA);
  Delay_us(Delaytime);
  return ReadValue;
}

/**
 * @brief 起始信号，SDA在SCL高电平期间由高变低
 *
 */
uint8_t i2c_Start(void)
{
  SDA(1);
  SCL(1);
  if (SDA_ReadState() == 0)
  {
    return Soft_I2C_BUS_BUSY; // 总线忙
  }
  SDA(0);
  SCL(0);                // 准备发送数据
  return Soft_I2C_READY; // 总线空闲
}

void i2c_Stop(void)
{
  SDA(0);
  SCL(1);
  SDA(1);
}

void i2c_SendACK(void)
{
  SDA(0); // 主机把应答位数据放到SDA线
  SCL(1); // 释放SCL，从机在SCL高电平期间，读取应答位
  SCL(0); // 拉低SCL，开始下一个时序模块
}

void i2c_SendNACK(void)
{
  SDA(1); // 主机把应答位数据放到SDA线
  SCL(1); // 释放SCL，从机在SCL高电平期间，读取应答位
  SCL(0); // 拉低SCL，开始下一个时序模块
}

/**
 * @brief i2c接收应答位
 *
 * 该函数用于接收从机发送的应答位。
 * 主机在接收数据后，需要读取从机发送的应答位，以确认数据是否正确接收。
 *
 * @return 返回接收到的应答位，0表示应答，1表示非应答。
 */
uint8_t i2c_ReceiveACK(void)
{
  uint8_t ACKBit;           // 定义应答位变量
  SDA(1);                   // 接收前，主机先确保释放SDA，避免干扰从机的数据发送
  SCL(1);                   // 释放SCL，主机机在SCL高电平期间读取SDA
  ACKBit = SDA_ReadState(); // 将应答位存储到变量里
  SCL(0);                   // 拉低SCL，开始下一个时序模块
  return ACKBit;            // 返回定义应答位变量
}

/**
 * @brief i2c发送一个字节
 *
 * @param Byte 要发送的字节数据
 * @return 返回接收到的应答位，0表示应答，1表示非应答
 */
uint8_t i2c_SendByte(uint8_t Byte)
{
  uint8_t i;
  for (i = 0; i < 8; i++) // 循环8次，主机依次发送数据的每一位
  {
    SDA(Byte & (0x80 >> i)); // 使用掩码的方式取出Byte的指定一位数据并写入到SDA线
    SCL(1);                  // 释放SCL，从机在SCL高电平期间读取SDA
    SCL(0);                  // 拉低SCL，主机开始发送下一位数据
  }
  return i2c_ReceiveACK(); // 返回接收到的应答位
}

/**
 * 函    数：i2c接收一个字节,发送NACK
 * 参    数：无
 * 返 回 值：接收到的一个字节数据
 */
uint8_t i2c_ReceiveByte_NACK(void)
{
  uint8_t i, Byte = 0x00;
  SDA(1);
  for (i = 0; i < 8; i++)
  {
    SCL(1);
    Byte <<= 1;
    if (SDA_ReadState())
    {
      Byte |= 0x01;
    }
    SCL(0);
  }
  i2c_SendNACK();
  return Byte; // 返回接收到的一个字节数据
}

/**
 * @brief 接收一个字节数据，接收完数据后，发送ACK
 *
 * @return Byte
 */
uint8_t i2c_ReceiveByte_ACK(void)
{
  uint8_t i, Byte = 0x00;
  SDA(1);
  for (i = 0; i < 8; i++)
  {
    SCL(1);
    Byte <<= 1;
    if (SDA_ReadState())
    {
      Byte |= 0x01;
    }
    SCL(0);
  }
  i2c_SendACK();
  return Byte;
}

/**
 * @brief 向I2C设备写入数据
 *
 * @param dev_addr 设备地址
 * @param reg_addr 寄存器地址
 * @param length 写入数据的长度
 * @param data 指向要写入的数据的指针
 * @return uint8_t 返回0表示成功，非0表示失败
 */
uint8_t i2c_WriteData(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
  uint8_t i, result = 0;
  i2c_Start();
  result = i2c_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);
  if (result != 0)
    return result;
  result = i2c_SendByte(reg_addr);
  if (result != 0)
    return result;
  for (i = 0; i < length; i++)
  {
    result = i2c_SendByte(data[i]);
    if (result != 0)
      return result;
  }
  i2c_Stop();
  return 0x00;
}

/**
 * @brief  从I2C设备读取数据
 *
 * @param  dev_addr      设备地址
 * @param  reg_addr      寄存器地址
 * @param  length        读取数据的长度
 * @param  read_databuf  存储读取数据的缓冲区指针
 *
 * @return uint8_t            操作结果，0表示成功，非0表示失败
 *
 * 该函数通过I2C协议从指定设备的指定寄存器读取指定长度的数据，并将读取的数据存储在提供的缓冲区中。
 * 如果在任何I2C操作过程中发生错误，函数将返回非0值以指示失败。
 */
uint8_t i2c_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *read_databuf)
{
  uint8_t result = 0;
  i2c_Start();
  result = i2c_SendByte(dev_addr << 1 | I2C_Direction_Transmitter);
  if (result != 0)
    return result;
  result = i2c_SendByte(reg_addr);
  if (result != 0)
    return result;
  i2c_Start();
  result = i2c_SendByte(dev_addr << 1 | I2C_Direction_Receiver);
  if (result != 0)
    return result;
  while (length)
  {
    if (length == 1)
      *read_databuf = i2c_ReceiveByte_NACK();
    else
      *read_databuf = i2c_ReceiveByte_ACK();
    read_databuf++;
    length--;
  }
  i2c_Stop();
  return 0x00;
}

/**
 * @brief  向IIC设备的寄存器连续写入数据，带超时重试设置，供mpu接口调用
 * @param  Address: IIC设备地址
 * @param  RegisterAddr: 寄存器地址
 * @param  RegisterLen: 要写入数据的长度
 * @param  RegisterValue: 要指向写入数据的指针
 * @retval 0正常，非0异常
 */
int i2c_WriteRegister(uint8_t slave_addr,
                      uint8_t reg_addr,
                      uint8_t len,
                      uint8_t *data_ptr)
{
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
  ret = 0;
  ret = i2c_WriteData(slave_addr, reg_addr, len, data_ptr);

  if (ret && retry_in_mlsec)
  {
    if (retries++ > 4)
      return ret;

    mdelay(retry_in_mlsec);
    goto tryWriteAgain;
  }
  return ret;
}

int i2c_ReadRegister(uint8_t slave_addr,
                     uint8_t reg_addr,
                     uint8_t len,
                     uint8_t *data_ptr)
{
  char retries = 0;
  int ret = 0;
  unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
  ret = 0;
  ret = i2c_ReadData(slave_addr, reg_addr, len, data_ptr);

  if (ret && retry_in_mlsec)
  {
    if (retries++ > 4)
      return ret;

    mdelay(retry_in_mlsec);
    goto tryReadAgain;
  }
  return ret;
}
