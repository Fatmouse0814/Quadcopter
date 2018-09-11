#include "bsp_i2c.h"

/**
  * @brief  IIC初始化
  * @param  无
  * @retval 无
  */
void IIC_Init(void)
{
	IIC_SDA = 1;
	IIC_SCL = 1;
}

/**
  * @brief  IIC开始传输
  * @param  void
  * @retval void
  */
void IIC_Start(void)
{
	
	SDA_OUT();
	IIC_SDA = 1;
	IIC_SCL = 1;
	IIC_Delay();
	IIC_SDA = 0;
	IIC_Delay();
		/*嵌住IIC，开始发送数据*/
	IIC_SCL = 0;

}

/**
  * @brief  IIC停止传输
  * @param  void
  * @retval void
  */
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SDA = 0;
	IIC_SCL = 0;
	IIC_Delay();	
	IIC_SCL = 1;
	IIC_SDA = 1;
	IIC_Delay();	
}

/**
  * @brief  IIC等待从机响应
  * @param  void
  * @retval 成功返回1
  */
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ErrorCount = 0; 
		
	SDA_IN();
	IIC_SDA = 1;IIC_Delay();
	IIC_SCL = 1;IIC_Delay();
	while(IIC_SDA_Read)
	{
		ErrorCount++;
		if(ErrorCount > 250)
		{
			IIC_Stop();
			return 0;
		}
	}
	IIC_SCL = 0;
	return 1;
}

/**
  * @brief  IIC发送响应
  * @param  void
  * @retval void
  */
void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
	IIC_Delay();
	IIC_SCL = 1;
	IIC_Delay();
	IIC_SCL = 0;
}

/**
  * @brief  IIC不发送响应
  * @param  void
  * @retval void
  */
void IIC_NAck(void)
{
IIC_SCL = 0;
SDA_OUT();
IIC_SDA = 1;
		IIC_Delay();
IIC_SCL = 1;
		IIC_Delay();
IIC_SCL = 0;
	
}

/**
  * @brief  IIC发送字节
  * @param  txd 需要发送的字节
  * @retval void
  */
void IIC_Send_Byte(uint8_t txd)
{
	uint8_t i;
	SDA_OUT();                               
	IIC_SCL = 0;
	for(i=0; i<8; i++)
	{	
		IIC_SDA = (txd&0x80)>>7;
		delay_us(1);
		txd <<= 1;
		IIC_SCL = 1;
		IIC_Delay();
		IIC_SCL = 0;
		IIC_Delay();
	}
}
/**
  * @brief  IIC接收字节
  * @param  ack 1.发送响应位 2.不发送响应位
  * @retval u8 返回收到的字节
  */
uint8_t IIC_Read_Byte(uint8_t ack)
{
	uint8_t i, rxd;
	SDA_IN();
	for( i=0; i<8; i++)
	{		
		IIC_SCL = 0;
		IIC_Delay();
		IIC_SCL = 1;
		rxd<<=1;
		if(IIC_SDA_Read) rxd++;	
		IIC_Delay();
	}
	if(ack) IIC_Ack();
		else IIC_NAck();
	IIC_SCL = 0;//修改过
	return rxd;
	
	
}

//写数据，成功返回0，失败返回0xff
int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data)
{
	IIC_Start();
    
 IIC_Send_Byte(dev_addr);
	if(IIC_Wait_Ack() == 0xff)
    {
        printf("error 2B\r\n");
        return 0xff;
    }
    
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack() == 0xff)
    {
        printf("error 2C\r\n");
        return 0xff;
    }

    IIC_Send_Byte(data);
    if(IIC_Wait_Ack()  == 0xff)
    {
        printf("error 2D\r\n");
        return 0xff;
    }

	 IIC_Stop();
    return 0;
}

//读数据，成功返回0，失败返回0xff
int IIC_ReadData(uint8_t dev_addr,uint8_t reg_addr,uint8_t *pdata,uint8_t count)
{
	  uint8_t i;

    IIC_Start();
	
    IIC_Send_Byte(dev_addr);
	  if(IIC_Wait_Ack() == 0xff)
    {
        printf("error 2F\r\n");
        return 0xff;
    }
    
    IIC_Send_Byte(reg_addr);
	  if(IIC_Wait_Ack() == 0xff)
    {
        printf("error 2G\r\n");
        return 0xff;
    }
	
    IIC_Start();
    
    IIC_Send_Byte(dev_addr+1);
	  if(IIC_Wait_Ack()  == 0xff)
    {
        printf("error 2H\r\n");
        return 0xff;
    }
    
    for(i=0;i<(count-1);i++)
    {
        *pdata= IIC_Read_Byte(1);
     //   IIC_Ack();
        pdata++;
    }

    *pdata=IIC_Read_Byte(0);
   //  IIC_NAck();
    
    IIC_Stop(); 
    
    return 0;    
}


/**
  * @brief  IIC延时函数
  * @param  无
  * @retval 无
  */
void IIC_Delay(void)
{
	delay_us(3);
}
