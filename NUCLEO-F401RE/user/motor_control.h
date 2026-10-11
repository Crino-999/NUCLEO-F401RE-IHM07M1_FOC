#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>

typedef struct
{
    float ta; /* PWM 通道 A 占空比 */
    float tb; /* PWM 通道 B 占空比 */
    float tc; /* PWM 通道 C 占空比 */
    float uq; /* dq 坐标系下的 q 轴电压 */
    float ud; /* dq 坐标系下的 d 轴电压 */
    float ualpha; /* α 坐标系下的电压 */
    float ubeta; /* β 坐标系下的电压 */
    float voltage_power_supply; /*电机驱动板供电电压 (V)*/
    uint32_t pole_pairs; /*电机极对数*/
    
    
} MotorControl_Config;

extern MotorControl_Config motorControlConfig;

void MotorControl_Init(void);

void MotorControl_Enable(char enable);

void vMotorControlTask(float iq, float id, float angle);

#endif 