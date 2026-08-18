#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"

/* AS5600 设备地址 (7-bit) */
#define AS5600_ADDR         0x36

/* AS5600 寄存器地址 */
#define AS5600_RAW_ANGLE_HI 0x0C  /* 角度高字节 */
#define AS5600_RAW_ANGLE_LO 0x0D  /* 角度低字节 */

/* 角度转换常量: 12-bit (0~4095) → 角度 (0~360) */
#define AS5600_ANGLE_SCALE  (360.0f / 4096.0f)

/**
 * @brief  读取 AS5600 原始角度值 (12-bit)
 * @return 0~4095, 失败返回 0
 */
uint16_t AS5600_ReadRawAngle(void);

/**
 * @brief  获取 AS5600 角度 (度)
 * @param  angle  输出角度值 (0.0~360.0)
 * @return 1=成功, 0=I2C通信失败
 */
uint8_t AS5600_GetAngle(float *angle);

#endif /* __AS5600_H */
