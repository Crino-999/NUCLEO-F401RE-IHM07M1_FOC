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
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- 开环强拖参数 ---- */
#define OL_UQ_ALIGN_V   0.50f    /* 对齐电压（V）：0.5 / Rs 3.19Ω ≈ 0.16A */
#define OL_ALIGN_MS     2000u    /* 对齐时长（ms） */
#define OL_UQ_RUN_V     0.80f    /* 旋转电压指令（V）：0.8 / Rs 3.19Ω ≈ 0.25A */
#define OL_FE_HZ        0.50f    /* 电气频率（Hz）：0.5 → 机械 4.3 rpm，1 圈约 14s */
#define OL_DIR         -1.0f     /* 旋转方向：+1 = 角度递增，-1 = 角度递减（反转）*/
#define OL_TICK_MS      1u       /* 控制周期（ms） */
#define OL_PRINT_MS     500u     /* 打印节流（ms） */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 开环强拖状态 */
static uint8_t  ol_state   = 0;      /* 0 = 对齐，1 = 旋转 */
static float    ol_theta_e = 0.0f;   /* 电角度（rad），即 Uref 的方向 */
static uint32_t ol_tState  = 0;      /* 状态切换时刻 */
static uint32_t ol_tAngle  = 0;      /* 上次角度积分时刻 */
static uint32_t ol_tCtrl   = 0;      /* 上次控制时刻 */
static uint32_t ol_tPrint  = 0;      /* 上次打印时刻 */
static uint32_t ol_revCnt  = 0;      /* 电气圈数 */
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
  MotorControl_Init();   /* 启动三路 PWM；此刻三相占空比均为 50%，零矢量，无输出电压 */

  /* 屏蔽 MCSDK 遗留中断：这些处理函数都建立在 MCboot() 初始化过的状态上，而我们没有调用它。
     其中 EXTI15_10 是 PC13 用户按键，误按会跳进 MC_StartMotor1() 去操作未初始化的 PWM 句柄。 */
  HAL_NVIC_DisableIRQ(ADC_IRQn);
  HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
  HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

  /* 启动自检：MOE=1 且三个通道已使能才可能有波形 */
  Printf("\r\nPWM selftest: MOE=%u CCER=0x%04X ARR=%u\r\n",
         (unsigned)((TIM1->BDTR & TIM_BDTR_MOE) ? 1u : 0u),
         (unsigned)TIM1->CCER, (unsigned)htim1.Init.Period);

  /* 使能功率桥：此处三相占空比仍是零矢量，不会产生电流 */
  MotorControl_Enable(1);
  Printf("EN=1 | 7 pole pairs, Rs=3.19ohm, Ke=3.3Vrms(LL)/kRPM, rated 2A\r\n");
  Printf("Phase 1 ALIGN: th_e=0deg, Uq=%.2fV, %ums\r\n", OL_UQ_ALIGN_V, (unsigned)OL_ALIGN_MS);
  Printf("Phase 2 SPIN : dir=%.0f, fe=%.2fHz (mech %.1f rpm), Uq=%.2fV\r\n\r\n",
         OL_DIR, OL_FE_HZ, OL_FE_HZ * 60.0f / 7.0f, OL_UQ_RUN_V);

  ol_tState = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (now - ol_tCtrl >= OL_TICK_MS)          /* 1 kHz 控制周期 */
    {
        ol_tCtrl = now;

        if (ol_state == 0)
        {
            /* 阶段 1 对齐：固定电角度，把转子吸到已知位置 */
            vMotorControlTask(OL_UQ_ALIGN_V, 0.0f, 0.0f);

            if (now - ol_tState >= OL_ALIGN_MS)
            {
                ol_state   = 1;
                ol_tAngle  = now;
                ol_theta_e = 0.0f;
                Printf(">> align done, start spinning\r\n");
            }
        }
        else
        {
            /* 阶段 2 旋转：角度按真实时间积分，打印抖动不会改变电气频率 */
            float dt = (float)(now - ol_tAngle) * 0.001f;
            ol_tAngle = now;

            ol_theta_e += OL_DIR * 2.0f * (float)M_PI * OL_FE_HZ * dt;
            if (ol_theta_e >= 2.0f * (float)M_PI)
            {
                ol_theta_e -= 2.0f * (float)M_PI;
                ol_revCnt++;
            }
            else if (ol_theta_e < 0.0f)
            {
                ol_theta_e += 2.0f * (float)M_PI;
                ol_revCnt++;
            }

            vMotorControlTask(OL_UQ_RUN_V, 0.0f, ol_theta_e);
        }
    }

    /* 每 500 ms 打一行；打印与 I2C 读取都是阻塞的，节流以免扰动 1kHz 控制节拍 */
    if (now - ol_tPrint >= OL_PRINT_MS)
    {
        ol_tPrint = now;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        /* 读 AS5600 得到的电角度（内部含一次 I2C 读取，约 0.5ms） */
        float enc_e = Encoder_GetElecAngle();
        float cmd_e = ol_theta_e * 180.0f / (float)M_PI;

        /* 编码器零点与电机磁轴零点之间的偏移未知，所以 d 会带一个恒定偏置。
           判断同步看的不是 d 的数值，而是 d 是否【恒定】：恒定 = 转子跟随磁场，
           持续漂移 = 丢步。 */
        float d = cmd_e - enc_e;
        while (d >  180.0f) { d -= 360.0f; }
        while (d < -180.0f) { d += 360.0f; }

        Printf("st:%u cmd:%6.1f enc_e:%6.1f d:%7.1f | duty:%5.3f,%5.3f,%5.3f | rev:%u\r\n",
               (unsigned)ol_state, cmd_e, enc_e, d,
               motorControlConfig.ta, motorControlConfig.tb, motorControlConfig.tc,
               (unsigned)ol_revCnt);
    }

#if 0   /* ---- 阶段2 第3步：SVPWM 数值扫描验证（已通过，见实验记录 20261011，保留备查） ---- */
    static uint32_t lastTick_scan = 0;
    static float theta_deg = 0.0f;   /* 合成矢量的电角度，0° → 345° 循环扫描 */

    if (HAL_GetTick() - lastTick_scan >= 200)   /* 每 200ms 打一行，一行对应一个角度 */
    {
        lastTick_scan = HAL_GetTick();
        /*LED2 闪烁*/
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

        /* ================= SVPWM 数值验证（阶段2 第3步） ================= */
        const float M = 1.0f;    /* 调制比 m = √3·|Uref|/Udc，1.0 = 线性区上限 */

        float u  = M / sqrtf(3.0f);              /* 归一化幅值 |Uref|/Udc */
        float th = theta_deg * (M_PI / 180.0f);
        motorControlConfig.ualpha = u * cosf(th) * motorControlConfig.voltage_power_supply;
        motorControlConfig.ubeta  = u * sinf(th) * motorControlConfig.voltage_power_supply;

        Svpwm_Calc(motorControlConfig.ualpha, motorControlConfig.ubeta,
                   motorControlConfig.voltage_power_supply,
                   &motorControlConfig.ta, &motorControlConfig.tb, &motorControlConfig.tc);

        /* 重构不变式：占空比 → 去共模 → Clarke，应等于 1.5×vα/vdc；正确时 err ≈ 0 */
        float un = (motorControlConfig.ta + motorControlConfig.tb + motorControlConfig.tc) / 3.0f;
        float r_alpha = 0.0f, r_beta = 0.0f;
        Transform_Clarke(motorControlConfig.ta - un,
                         motorControlConfig.tb - un,
                         motorControlConfig.tc - un,
                         &r_alpha, &r_beta);
        float e_alpha = 1.5f * motorControlConfig.ualpha / motorControlConfig.voltage_power_supply;
        float e_beta  = 1.5f * motorControlConfig.ubeta  / motorControlConfig.voltage_power_supply;
        float err = sqrtf((r_alpha - e_alpha) * (r_alpha - e_alpha)
                        + (r_beta  - e_beta ) * (r_beta  - e_beta ));

        /* sec 是扫描角度推算的参考扇区，仅便于与手算表对照，不代表算法内部结果 */
        Printf("th:%3.0f sec:%d | duty:%5.3f,%5.3f,%5.3f | err:%9.2e | rec:%6.3f,%6.3f exp:%6.3f,%6.3f\r\n",
               theta_deg, (int)(theta_deg / 60.0f) + 1,
               motorControlConfig.ta, motorControlConfig.tb, motorControlConfig.tc,
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
#endif
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
