/*
 * mpu_6050.c
 *
 *  Created on: 6 abr 2026
 *      Author: octav
 */

#include "../bsp/inc/mpu_6050.h"
void MPU6050_Init(MPU6050_Handle_t *pMPU6050Handle)
{
    uint8_t config[2];

    config[0] = MPU6050_ADDR_PWR_MGMT_1; // PWR_MGMT_1
    config[1] = 0x00;                    // Wake up MPU6050

    I2C_MasterSendData(pMPU6050Handle->pI2CHandle, config, 2, MPU6050_I2C_DEVICE_ADDR, I2C_ENABLE_SR);
}
void MPU6050_ReadGyroXYZ(MPU6050_Handle_t *pMPU6050Handle) {}
uint8_t MPU6050_IsMoving(MPU6050_Handle_t *pMPU6050Handle, int16_t threshold) { return 0; }

uint16_t MPU6050_ReadTemp(MPU6050_Handle_t *pMPU6050Handle)
{
    uint8_t rx[2];
    uint16_t temp_raw;

    I2C_MasterSendData(pMPU6050Handle->pI2CHandle, (uint8_t *)MPU6050_TEMP_OUT_H, 1, MPU6050_I2C_DEVICE_ADDR, I2C_ENABLE_SR);

    I2C_MasterReceiveData(pMPU6050Handle->pI2CHandle, rx, 2, MPU6050_I2C_DEVICE_ADDR, I2C_DISABLE_SR);

    temp_raw = (uint16_t)((rx[0] << 8) | rx[1]);
    return temp_raw;
}
uint8_t MPU6050_WhoAmI(MPU6050_Handle_t *pMPU6050Handle)
{
    uint8_t id;
    I2C_MasterSendData(pMPU6050Handle->pI2CHandle, (uint8_t *)MPU6050_ADDR_WHOAM_I, 1, MPU6050_I2C_DEVICE_ADDR, I2C_ENABLE_SR);

    I2C_MasterReceiveData(pMPU6050Handle->pI2CHandle, &id, 1, MPU6050_I2C_DEVICE_ADDR, I2C_DISABLE_SR);

    return id;
}
