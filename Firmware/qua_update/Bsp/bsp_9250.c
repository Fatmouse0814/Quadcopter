#include "bsp_9250.h"

//定义输出速度
#define DEFAULT_MPU_HZ  (100)		//100Hz

#define q30  1073741824.0f
#define q16  65536.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};
//磁力计方向设置
static signed char comp_orientation[9] = { 0, 1, 0,
                                           1, 0, 0,
                                           0, 0,-1};

/**
  * @brief  MPU连续写入
  * @param  slave_addr 从机地址
	*					reg_addr 寄存器位置
	*					length 读取的长度
	*					data 写入的数据
  * @retval 成功返回0
  */
uint8_t MPU_Write_Len(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data){
	
	uint8_t i;
	IIC_Start();
	
	IIC_Send_Byte((slave_addr<<1) | 0);
	
	if(!IIC_Wait_Ack()){
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte((reg_addr));
	IIC_Wait_Ack();
	
	for(i=0; i<length; i++){
		
		IIC_Send_Byte(data[i]);
		if(!IIC_Wait_Ack()){
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
	
}
/**
  * @brief  MPU连续读取
  * @param  slave_addr 从机地址
	*					reg_addr 寄存器位置
	*					length 读取的长度
	*					data 返回的数据
  * @retval 成功返回0
  */
uint8_t MPU_Read_Len(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data){
	
	IIC_Start();
	IIC_Send_Byte((slave_addr<<1) | 0);
	if(!IIC_Wait_Ack()){
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg_addr);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte((slave_addr<<1 )| 1);
	IIC_Wait_Ack();
	while(length){
		if(length == 1) *data = IIC_Read_Byte(0);
		else *data = IIC_Read_Byte(1);
		data++;
		length--;
	}
	IIC_Stop();
	return 0;
}

/**
  * @brief  MPU写入一个字节
  * @param  reg 寄存器位置 
	*					data 需要发送的字节
  * @retval 成功返回0
  */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data){
	
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1) | 0);
	if(!IIC_Wait_Ack()){
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	if(!IIC_Wait_Ack()){
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}

/**
  * @brief  MPU读取一个字节
  * @param  reg 寄存器位置
  * @retval rxd 接收到的字节
  */
uint8_t MPU_Read_Byte(uint8_t reg){
	
	uint8_t rxd;
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1) | 0);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1) | 1);
	IIC_Wait_Ack();
	rxd = IIC_Read_Byte(0);
	IIC_Stop();
	return rxd;
	
}
/**
  * @brief  mpu9250自检函数
  * @param  无
  * @retval 0 正常
  */
uint8_t run_self_test(void)
{
	int result;
	long gyro[3], accel[3]; 
	result = mpu_run_6500_self_test(gyro, accel,0);
	if (result == 0x7) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
        unsigned short accel_sens;
		float gyro_sens;

		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);

		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;

		dmp_set_accel_bias(accel);
		return 0;
	}else return 1;
}

/**
  * @brief  mpu9250方向转换
  * @param  略
  * @retval 略
  */
unsigned short inv_row_2_scale(const signed char *row)
{
		unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

//mpu6050,dmp初始化
//返回值:0,正常
//    其他,失败
uint8_t mpu_dmp_init(void)
{
	uint8_t res=0;
  struct int_param_s int_param;
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
  unsigned short compass_fsr;
    
	IIC_Init(); 		        //初始化IIC总线
	if(mpu_init(&int_param)==0)	//初始化MPU9250
	{	 		
        res=inv_init_mpl();     //初始化MPL
        if(res)return 1;
        inv_enable_quaternion();
        inv_enable_9x_sensor_fusion();
        inv_enable_fast_nomot();
        inv_enable_gyro_tc();
        inv_enable_vector_compass_cal();
        inv_enable_magnetic_disturbance();
        inv_enable_eMPL_outputs();
        res=inv_start_mpl();    //开启MPL
        if(res)return 1;
		res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL|INV_XYZ_COMPASS);//设置所需要的传感器
		if(res)return 2; 
		res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   //设置FIFO
		if(res)return 3; 
		res=mpu_set_sample_rate(DEFAULT_MPU_HZ);	            //设置采样率
		if(res)return 4; 
        res=mpu_set_compass_sample_rate(1000/COMPASS_READ_MS);  //设置磁力计采样率
        if(res)return 5;
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        mpu_get_compass_fsr(&compass_fsr);
        inv_set_gyro_sample_rate(1000000L/gyro_rate);
        inv_set_accel_sample_rate(1000000L/gyro_rate);
        inv_set_compass_sample_rate(COMPASS_READ_MS*1000L);
        inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)gyro_fsr<<15);
        inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_orientation),(long)accel_fsr<<15);
        inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(comp_orientation),(long)compass_fsr<<15);
            
            
		res=dmp_load_motion_driver_firmware();		             //加载dmp固件
		if(res)return 6; 
		res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
		if(res)return 7; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	            //设置dmp功能
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 8; 
		res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);	//设置DMP输出速率(最大不超过200Hz)
		if(res)return 9;   
//		res=run_self_test();		//自检
		if(res)return 10;    
		res=mpu_set_dmp_state(1);	//使能DMP
		if(res)return 11;     
	//	printf("MPU9250InitSuccessful！！！\r\n");
	
	}
	else return 12;
	return 0;
}
/**
  * @brief  获取DMP处理后的数据
  * @param  
  * @retval 
  */
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1;	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//计算得到俯仰角/横滚角/航向角
		//*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		//*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3+180;	//yaw
	}else return 2;
	return 0;
}

/**
  * @brief  获取MPL处理后数据
  * @param  pitch 俯仰角
  * @param  roll 横滚角
  * @param  yaw 航向角+180
  * @retval 0 正常
  */
uint8_t mpu_mpl_get_data(float *pitch,float *roll,float *yaw,float *gyro_x,float *gyro_y,float *gyro_z)
{
	unsigned long sensor_timestamp,timestamp;
	short gyro[3], accel_short[3],compass_short[3],sensors;
	unsigned char more;
	long compass[3],accel[3],quat[4],temperature; 
    long data[9];
    int8_t accuracy;
	
	if(dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors,&more))return 1;	 
		
    if(sensors&INV_XYZ_GYRO)
    {
        inv_build_gyro(gyro,sensor_timestamp);          //把新数据发送给MPL
        mpu_get_temperature(&temperature,&sensor_timestamp);
        inv_build_temp(temperature,sensor_timestamp);   //把温度值发给MPL，只有陀螺仪需要温度值
    }
    
    if(sensors&INV_XYZ_ACCEL)
    {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel,0,sensor_timestamp);      //把加速度值发给MPL
    }
    
    if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) 
    {
        compass[0]=(long)compass_short[0];
        compass[1]=(long)compass_short[1];
        compass[2]=(long)compass_short[2];
        inv_build_compass(compass,0,sensor_timestamp); //把磁力计值发给MPL
    }
    inv_execute_on_data();
    inv_get_sensor_type_euler(data,&accuracy,&timestamp);
    
    *roll  = (data[0]/q16);
    *pitch = -(data[1]/q16);
    *yaw   = data[2] / q16+ 180.0f ;
		*gyro_x= gyro[0]/16.4f;
		*gyro_y= gyro[1]/16.4f;
		*gyro_z= gyro[2]/16.4f;
	return 0;
}

/**
  * @brief  空函数
  * @param  略
  * @retval 略
  */
void mget_ms(unsigned long *time)
{
}
