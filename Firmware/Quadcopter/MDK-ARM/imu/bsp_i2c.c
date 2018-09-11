#include "bsp_i2c.h"

/**
  * @brief  IIC��ʼ��
  * @param  ��
  * @retval ��
  */
void IIC_Init(void)
{
	IIC_SDA = 1;
	IIC_SCL = 1;
}

/**
  * @brief  IIC��ʼ����
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
		/*ǶסIIC����ʼ��������*/
	IIC_SCL = 0;

}

/**
  * @brief  IICֹͣ����
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
  * @brief  IIC�ȴ��ӻ���Ӧ
  * @param  void
  * @retval �ɹ�����1
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
  * @brief  IIC������Ӧ
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
  * @brief  IIC��������Ӧ
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
  * @brief  IIC�����ֽ�
  * @param  txd ��Ҫ���͵��ֽ�
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
  * @brief  IIC�����ֽ�
  * @param  ack 1.������Ӧλ 2.��������Ӧλ
  * @retval u8 �����յ����ֽ�
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
	IIC_SCL = 0;//�޸Ĺ�
	return rxd;
	
	
}

//д���ݣ��ɹ�����0��ʧ�ܷ���0xff
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

//�����ݣ��ɹ�����0��ʧ�ܷ���0xff
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
  * @brief  IIC��ʱ����
  * @param  ��
  * @retval ��
  */
void IIC_Delay(void)
{
	delay_us(3);
}
