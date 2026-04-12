/*
 * mpu_6050.h
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#ifndef INC_MPU_6050_H_
#define INC_MPU_6050_H_

#include "../Drivers/inc/stm32f411xx.h"
/* Application */

#define MPU6050_I2C             I2C1
#define MPU6050_I2C_ACKCTRL     I2C_ACK_ENABLE
#define MPU6050_I2C_DEVICE_ADDR MPU_6050_I2C_ADDR_AD1
#define MPU6050_I2C_SCLSPEED    I2C_SCL_SPEED_SM
#define MPU6050_I2C_DUTYCYCLE   I2C_FM_DUTY_2

/* MPU6050 CONTROL REGISTERS */
#define MPU6050_ADDR_SMPLRT_DIV  0x19
#define MPU6050_ADDR_CONFIG      0x1A
#define MPU6050_ADDR_GYRO_CONFIG 0x1B

#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42

#define MPU6050_ADD_GYRO_XOUT_H 0x43
#define MPU6050_ADD_GYRO_XOUT_L 0x44
#define MPU6050_ADD_GYRO_YOUT_H 0x45
#define MPU6050_ADD_GYRO_YOUT_L 0x46
#define MPU6050_ADD_GYRO_ZOUT_H 0x47
#define MPU6050_ADD_GYRO_ZOUT_L 0x48

#define MPU6050_ADDR_PWR_MGMT_1 0x6B
#define MPU6050_ADDR_PWR_MGMT_2 0x6C
#define MPU6050_ADDR_WHOAM_I    0x75

/*Control Struct*/
typedef struct
{
    uint8_t deviceAddr;          // 0x68 o 0x69
    uint8_t gyro_full_scale;     // ±250, ±500, ±1000, ±2000 dps
    uint8_t sample_rate_divider; // SMPLRT_DIV
    uint8_t dlpf_config;         // Enable DLPF
} MPU6050_Config_t;

typedef struct
{
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;
} MPU6050_GyroRaw_t;

typedef struct
{
    I2C_Handle_t *pI2CHandle;
    MPU6050_Config_t config;
    MPU6050_GyroRaw_t gyroRaw;
} MPU6050_Handle_t;

/* I2C MPU6050 ADDRESS*/
#define MPU_6050_I2C_ADDR_AD0 0x68
#define MPU_6050_I2C_ADDR_AD1 0x69

#define MPU6050_GYRO_SCALE_250  250
#define MPU6050_GYRO_SCALE_500  500
#define MPU6050_GYRO_SCALE_1000 1000
#define MPU6050_GYRO_SCALE_2000 2000

#define MPU6050_DLPF_CFG_3 3

/*Prototype Functions*/

void MPU6050_Init(MPU6050_Handle_t *pMPU6050Handle);
void MPU6050_WriteReg(MPU6050_Handle_t *pMPU6050Handle);
uint8_t MPU6050_WhoAmI(MPU6050_Handle_t *pMPU6050Handle);

uint16_t MPU6050_ReadTemp(MPU6050_Handle_t *pMPU6050Handle);

void MPU6050_ReadGyroXYZ(MPU6050_Handle_t *pMPU6050Handle);
uint8_t MPU6050_IsMoving(MPU6050_Handle_t *pMPU6050Handle, int16_t threshold);

#endif /* INC_MPU_6050_H_ */
