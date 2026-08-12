
/**
  ******************************************************************************
  * @file    stm32f4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32F4 Family.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32F4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_config.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);

/**
  * @brief  This function handles ADC interrupt request.
  * @param  None
  */
/* ====== ADC 中断: FOC 高频环入口(16kHz) ======
 * 触发源: TIM1 CH4 在 PWM 周期内某时刻产生 ADC 注入外部触发,
 *         ADC 完成两相电流注入转换后置位 JEOS 标志并触发本中断。
 * 此处调用 TSK_HighFrequencyTask() -> FOC_CurrControllerM1(),
 * 即每个 PWM 周期执行一次完整的电流环 + SVPWM。 */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */

  /* Shared IRQ management - begin */

  if(LL_ADC_IsActiveFlag_JEOS(ADC1))
  {
    /* Clear Flags */
    ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);
    TSK_HighFrequencyTask();          /*GUI, this section is present only if DAC is disabled*/
  }
  else
  {
    /* Nothing to do */
  }

  /* USER CODE BEGIN HighFreq */

  /* USER CODE END HighFreq  */

  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
/* ====== TIM1 更新中断: 每个 PWM 周期开始时触发(16kHz) ======
 * 作用: 调用 R3_1_TIMx_UP_IRQHandler,按当前扇区重配 ADC 注入序列,
 *       决定本周期采哪两相电流。先于 ADC 注入触发执行。 */
void TIMx_UP_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_M1_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(TIM1);
  R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);

  /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_M1_IRQn 1 */
}

/**
  * @brief  This function handles first motor BRK interrupt.
  * @param  None
  */
/* ====== TIM1 刹车中断: 硬件过流/过压保护 ======
 * 触发后硬件自动封锁 PWM 输出(MOE=0),这里做软件善后并重新使能输出。 */
void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_DP_Handler(&PWM_Handle_M1._Super);
    LL_TIM_EnableAllOutputs( TIM1 );

  }
  else
  {
    /* Nothing to do */
  }

  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
