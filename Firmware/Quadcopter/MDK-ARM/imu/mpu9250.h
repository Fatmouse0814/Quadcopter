#ifndef __MPU9250_H
#define __MPU9250_H

#include "delay.h"
#include "usart.h"
#include "inv_mpu.h"
#include "mpl.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "data_builder.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "bsp_i2c.h"

#define MPU_ADDR 0x68

void mget_ms(unsigned long *time);
unsigned short inv_row_2_scale(const signed char *row);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
uint8_t run_self_test(void);
uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw);
//uint8_t mpu_mpl_get_data(float *pitch,float *roll,float *yaw);
uint8_t mpu_mpl_get_data(float *pitch,float *roll,float *yaw,float *gyro_x,float *gyro_y,float *gyro_z);
uint8_t run_self_test(void);

unsigned short inv_row_2_scale(const signed char *row);

uint8_t mpu_dmp_init(void);

uint8_t mpu_dmp_get_data(float *pitch,float *roll,float *yaw);


uint8_t MPU_Write_Len(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data);

uint8_t MPU_Read_Len(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data);

uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data);

uint8_t MPU_Read_Byte(uint8_t reg);
#endif
