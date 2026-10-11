/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "transform.h"
#include "svpwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  // MX_MotorControl_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t lastTick = 0;
    static float theta_deg = 0.0f;   /* 合成矢量的电角度，0° → 345° 循环扫描 */

    if (HAL_GetTick() - lastTick >= 200)   /* 每 200ms 打一行，一行对应一个角度 */
    {
        lastTick = HAL_GetTick();
        /*LED2 闪烁*/
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        /* ================= SVPWM 数值验证（阶段2 第3步） ================= */
        const float VDC_NOM = 12.0f;   /* 标称母线电压：纯数值验证用，与实际是否上电无关 */
        const float M       = 1.0f;    /* 调制比 m = √3·|Uref|/Udc，1.0 = 线性区上限 */
        const float TS      = 1.0f;    /* 归一化周期，与 svpwm.c 内部单位一致 */
        uint32_t    ARR     = htim1.Init.Period;   /* 取实际 ARR（2625 @16kHz/84MHz），不硬编码 */

        float u  = M / sqrtf(3.0f);              /* 归一化幅值 |Uref|/Udc */
        float th = theta_deg * (M_PI / 180.0f);
        float v_alpha = u * cosf(th) * VDC_NOM;
        float v_beta  = u * sinf(th) * VDC_NOM;

        uint32_t ccr_a = 0, ccr_b = 0, ccr_c = 0;
        Svpwm_Calc(v_alpha, v_beta, VDC_NOM, TS, ARR, &ccr_a, &ccr_b, &ccr_c);

        /* 重构不变式：CCR → 占空比 → 平均相电压(标幺) → Clarke → 应等于 1.5×v_alpha/vdc
           正确时 err ≈ 0，这是本步唯一的通过判据 */
        float dA = (float)ccr_a / (float)ARR;
        float dB = (float)ccr_b / (float)ARR;
        float dC = (float)ccr_c / (float)ARR;
        float un = (dA + dB + dC) / 3.0f;
        float r_alpha = 0.0f, r_beta = 0.0f;
        Transform_Clarke(dA - un, dB - un, dC - un, &r_alpha, &r_beta);
        float e_alpha = 1.5f * v_alpha / VDC_NOM;
        float e_beta  = 1.5f * v_beta  / VDC_NOM;
        float err = sqrtf((r_alpha - e_alpha) * (r_alpha - e_alpha)
                        + (r_beta  - e_beta ) * (r_beta  - e_beta ));

        /* sec 是由扫描角度推算的参考扇区，仅便于与手算表对照，不代表算法内部结果 */
        Printf("th:%3.0f sec:%d | ccr:%4u,%4u,%4u | err:%5.3f | rec:%6.3f,%6.3f exp:%6.3f,%6.3f\r\n",
               theta_deg, (int)(theta_deg / 60.0f) + 1,
               (unsigned)ccr_a, (unsigned)ccr_b, (unsigned)ccr_c,
               err, r_alpha, r_beta, e_alpha, e_beta);

        theta_deg += 15.0f;
        if (theta_deg >= 360.0f) { theta_deg -= 360.0f; }
        /* =============================================================== */

        /* ---- 阶段2 第2步：坐标变换验证（已通过，保留备查） ----
        float I = 1.0f;
        float theta = Encoder_GetElecRad();
        float ia = I * cosf(theta);
        float ib = I * cosf(theta - 2.0f*M_PI/3.0f);
        float ic = I * cosf(theta + 2.0f*M_PI/3.0f);
        float i_alpha, i_beta, id, iq;
        Transform_Clarke(ia, ib, ic, &i_alpha, &i_beta);
        float sin_val, cos_val;
        Encoder_GetSinCos(&sin_val, &cos_val);
        Transform_Park(i_alpha, i_beta, sin_val, cos_val, &id, &iq);
        Printf("the:%6.1f  ia:%5.2f ib:%5.2f  id:%5.2f iq:%5.2f\r\n", theta * 180.0f / M_PI, i_alpha, i_beta, id, iq);
        ---- 阶段2 第2步验证结束 ---- */
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
