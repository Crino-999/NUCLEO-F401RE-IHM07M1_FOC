#include "as5600.h"
#include "i2c.h"

#define I2C_TIMEOUT  100  /* I2C 通信超时 (ms) */

/* HAL I2C 句柄在 i2c.c 中定义 */
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief  通过硬件 I2C1 读取 AS5600 角度传感器 (12-bit)
 *
 * HAL_I2C_Mem_Read 内部处理:
 *   Start → DevAddr(W) + RegAddr → Restart → DevAddr(R) + Data → Stop
 *
 * @return 0~4095, 通信失败返回 0
 */
uint16_t AS5600_ReadRawAngle(void)
{
    uint8_t data[2];

    /* 从 0x0C 寄存器连续读取 2 字节 (高字节在前, 低字节在后) */
    if (HAL_I2C_Mem_Read(&hi2c1,
                         (uint16_t)(AS5600_ADDR << 1),   /* HAL 需要左移 1 位 */
                         AS5600_RAW_ANGLE_HI,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         2,
                         I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    return ((uint16_t)data[0] << 8) | data[1];
}

/**
 * @brief  获取 AS5600 角度 (度)
 * @param  angle  输出角度值 (0.0~360.0)
 * @return 1=成功, 0=I2C通信失败
 */
uint8_t AS5600_GetAngle(float *angle)
{
    uint8_t data[2];
    uint16_t raw;
    HAL_StatusTypeDef ret;

    if (angle == NULL)
        return 0;

    /* 从 0x0C 寄存器连续读取 2 字节 (高字节在前, 低字节在后) */
    ret = HAL_I2C_Mem_Read(&hi2c1,
                           (uint16_t)(AS5600_ADDR << 1),
                           AS5600_RAW_ANGLE_HI,
                           I2C_MEMADD_SIZE_8BIT,
                           data,
                           2,
                           I2C_TIMEOUT);
    if (ret != HAL_OK)
    {
        *angle = 0.0f;
        return 0;
    }

    raw = ((uint16_t)data[0] << 8) | data[1];
    *angle = (float)raw * AS5600_ANGLE_SCALE;
    return 1;
}
