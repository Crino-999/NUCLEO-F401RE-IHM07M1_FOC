#include "motor_control.h"
#include "math.h"
#include "main.h"
#include "tim.h"
#include "parameters_conversion.h"
#include "transform.h"
#include "svpwm.h"

MotorControl_Config motorControlConfig;

void MotorControl_Init(void)
{
    /* 初始化电机控制相关外设 */
    motorControlConfig.voltage_power_supply = 12.0f; /* 设置电机驱动板供电电压 */
    motorControlConfig.pole_pairs = 7; /* 设置电机极对数 */
    motorControlConfig.ta = 0.5f; /* 初始化 PWM 通道 A 的占空比 */
    motorControlConfig.tb = 0.5f; /* 初始化 PWM 通道 B 的占空比 */
    motorControlConfig.tc = 0.5f; /* 初始化 PWM 通道 C 的占空比 */
    motorControlConfig.uq = 0.0f; /* 初始化 dq 坐标系下的 q 轴电压 */
    motorControlConfig.ud = 0.0f; /* 初始化 dq 坐标系下的 d 轴电压 */
    motorControlConfig.ualpha = 0.0f; /* 初始化 α 坐标系下的电压 */
    motorControlConfig.ubeta = 0.0f; /* 初始化 β 坐标系下的电压 */

    /* 启动 TIM1 三个通道的 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}
void MotorControl_Enable(char enable)
{
    if(enable)
    {/* 使能电机驱动板 */
        HAL_GPIO_WritePin(M1_PWM_EN_U_GPIO_Port,M1_PWM_EN_U_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_PWM_EN_V_GPIO_Port,M1_PWM_EN_V_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_PWM_EN_W_GPIO_Port,M1_PWM_EN_W_Pin,GPIO_PIN_SET);
    }
    else
    {/* 关闭电机驱动板 */
        HAL_GPIO_WritePin(M1_PWM_EN_U_GPIO_Port,M1_PWM_EN_U_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_PWM_EN_V_GPIO_Port,M1_PWM_EN_V_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_PWM_EN_W_GPIO_Port,M1_PWM_EN_W_Pin,GPIO_PIN_RESET);
    }
}
void vMotorControlTask(float iq, float id, float angle)
{
    /* iq 和 id 转换为 uq 和 ud */
    //临时措施，开环时直接将 iq 和 id 赋值给 uq 和 ud
    motorControlConfig.uq = iq;
    motorControlConfig.ud = id;
    /* RevPark 变换 */
    Transform_InversePark(motorControlConfig.ud, motorControlConfig.uq,sinf(angle),
         cosf(angle), &motorControlConfig.ualpha, &motorControlConfig.ubeta);
    /* SVPWM 输出 */
    Svpwm_Calc(motorControlConfig.ualpha, motorControlConfig.ubeta, motorControlConfig.voltage_power_supply,
               &motorControlConfig.ta, &motorControlConfig.tb, &motorControlConfig.tc);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(motorControlConfig.ta * (float)htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(motorControlConfig.tb * (float)htim1.Init.Period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(motorControlConfig.tc * (float)htim1.Init.Period));
}