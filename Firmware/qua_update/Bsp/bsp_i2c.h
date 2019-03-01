#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__

#include "sys.h"
#include "delay.h"

#define SDA_IN()	  {GPIOB->MODER &= ~(3 << (6*2));GPIOB->MODER |= 0<< (6*2);}
#define SDA_OUT() 	{GPIOB->MODER &= ~(3 << (6*2));GPIOB->MODER |= 1<< (6*2);}

#define IIC_SCL PBout(7)
#define IIC_SDA PBout(6)
#define IIC_SDA_Read PBin(6)




void IIC_Init(void);
void IIC_Delay(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);
int IIC_ReadData(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t count);
int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data);

#endif
