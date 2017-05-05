#ifndef __HALMPU9250_H
#define __HALMPU9250_H

#include "stm32f10x.h"


#define CONFIG          (0x1A)
#define GYRO_CONFIG     (0x1B)
#define ACCEL_CONFIG    (0x1C)
#define ACCEL_CONFIG_2  (0x1D)

#define INT_PIN_CFG     (0x37)

#define ACCEL_XOUT_H    (0x3B)
#define ACCEL_XOUT_L    (0x3C)
#define ACCEL_YOUT_H    (0x3D)
#define ACCEL_YOUT_L    (0x3E)
#define ACCEL_ZOUT_H    (0x3F)
#define ACCEL_ZOUT_L    (0x40)

#define TEMP_OUT_H      (0x41)
#define TEMP_OUT_L      (0x42)

#define GYRO_XOUT_H     (0x43)
#define GYRO_XOUT_L     (0x44)
#define GYRO_YOUT_H     (0x45)
#define GYRO_YOUT_L     (0x46)
#define GYRO_ZOUT_H     (0x47)
#define GYRO_ZOUT_L     (0x48)

#define USER_CTRL       (0x6A)
#define PWR_MGMT_1      (0x6B)
#define PWR_MGMT_2      (0x6C)
#define REG_WHO_AM_I    (0x75)

#define I2C_SLV0_ADDR   (0x25)
#define I2C_SLV0_REG    (0x26)
#define I2C_SLV0_CTRL   (0x27)
#define I2C_SLV0_DO     (0x63)

#define I2C_SLV4_ADDR   (0x31)
#define I2C_SLV4_REG    (0x32)
#define I2C_SLV4_DO     (0x33)
#define I2C_SLV4_CTRL   (0x34)
#define I2C_SLV4_DI     (0x35)
#define I2C_MST_STATUS  (0x36)


extern void halMPU9250Init(void);

extern void halMPU9250RdReg(uint8_t addr, uint8_t* data, uint8_t len);
extern void halMPU9250WrReg(uint8_t addr, uint8_t data);

extern uint8_t halMPU9250RdCompassX(int16_t* cx);
extern uint8_t halMPU9250RdCompassY(int16_t* cy);
extern uint8_t halMPU9250RdCompassZ(int16_t* cz);

extern float halMPU9250RdAccelX(void);
extern float halMPU9250RdAccelY(void);
extern float halMPU9250RdAccelZ(void);

extern float halMPU9250RdGyroX(void);
extern float halMPU9250RdGyroY(void);
extern float halMPU9250RdGyroZ(void);



#endif



