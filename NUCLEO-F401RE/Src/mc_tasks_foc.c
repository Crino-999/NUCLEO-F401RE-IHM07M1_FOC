
/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
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
  * @ingroup MCTasksFOC
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

 /** @defgroup ConvFOC Conventional FOC
  *
  * @brief
  *
  * @{
  */

  /** @defgroup MCCockpit MC Cockpit
  *
  * @brief
  *
  * @{
  */

/** @addtogroup ConvFOC
  * @{
  */

/** @addtogroup	MCCockpit
  * @{
  */

/** @addtogroup	MCTasksFOC
  * @{
  */

/** @defgroup MCTasksFOC Motor Control Tasks for FOC
  *
  * @brief FOC legacy Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/

/* USER CODE END Private define */

/* Private variables----------------------------------------------------------*/

/* Angle and Speed estimation for STOPLL and STOCORDIC deactivation for IDLE=0, FAULT_NOW=10, FAULT_OVER=11
and CHARGE_BOOT_CAP=16 */
bool AngleSpeedEstimation[STATE_ENUM_COUNT]={false, true, true, true, true, true, true, true, true, true, false, \
                              false, true, true, true, true, false, true, true, true, true, true, true};

static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);

MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);

void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  */
__weak void FOC_Init(void)
{

  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

    /* ====== FOC 子系统初始化:依次建立各功能组件 ====== */
    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    /* PWM 与电流采样(R3_1 三电阻采样):配置 TIM1 中心对齐 PWM、ADC 注入触发 */
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_1_Init(&PWM_Handle_M1);

    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    /* 速度环 PI(外环,1kHz) */
    PID_HandleInit(&PIDSpeedHandle_M1);

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    /* 反电势观测器 STO-PLL:无感方案的核心,由电流/电压估算转子角度与转速 */
    STO_PLL_Init (&STO_PLL_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    /* 速度与转矩控制器:管理速度参考斜坡,并把速度误差送入速度 PI */
    STC_Init(pSTC[M1],&PIDSpeedHandle_M1, &STO_PLL_M1._Super);

    /**************************************/
    /*   Rev-up component initialization  */
    /**************************************/
    /* 开环启动控制:管理 IDLE->RUN 的开环拖动阶段曲线 */
    RUC_Init(&RevUpControlM1, pSTC[M1], &VirtualSpeedSensorM1, &STO_M1, pwmcHandle[M1]);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    /* 电流环 PI(内环,16kHz):Iq 控制转矩,Id 控制磁链 */
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    /*******************************************************/
    /*   Flux weakening component initialization           */
    /*******************************************************/
    /* 弱磁控制:高速时降低 Id 以削弱磁链,扩展调速范围 */
    PID_HandleInit(&PIDFluxWeakeningHandle_M1);
    FW_Init(pFW[M1],&PIDSpeedHandle_M1,&PIDFluxWeakeningHandle_M1);

    /*******************************************************/
    /*   Feed forward component initialization             */
    /*******************************************************/
    /* 电压前馈:根据电机模型提前给出 Vd/Vq 前馈量,提升动态性能 */
    FF_Init(pFF[M1],&(BusVoltageSensor_M1._Super),pPIDId[M1],pPIDIq[M1]);

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    STC_Clear(pSTC[M1]);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;

    MCI_ExecSpeedRamp(&Mci[M1],
    STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /* First command to STC */

    /* USER CODE BEGIN MCboot 2 */

    /* USER CODE END MCboot 2 */
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  R3_1_SwitchOffPWM(pwmcHandle[motor]);

  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);

  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  /* 中频任务入口: 在 SysTick 中以 1kHz 频率调用(SPEED_LOOP_FREQUENCY_HZ=1000)
   * 这里先更新平均转速与电功率统计,再按状态机推进 */
  (void)STO_PLL_CalcAvrgMecSpeedUnit(&STO_PLL_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      /* ====== 电机控制状态机 ====== */
      switch (Mci[M1].State)
      {

        /* IDLE: 待机,等待启动命令。收到 MCI_START 后进入电流偏置校准/充电自举 */
        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
              RUC_Clear(&RevUpControlM1, MCI_GetImposedMotorDirection(&Mci[M1]));
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              /* Calibration already done. Enables only TIM channels */
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);

              R3_1_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* Nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        /* OFFSET_CALIB: 电流偏置校准。PWM 关断时采 ADC 零电流,得到三相偏置
         * (用于消除运放/ADC 的零点漂移),完成后进入 CHARGE_BOOT_CAP */
        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_1_TurnOnLowSides(pwmcHandle[M1],M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* Nothing to be done, FW waits for offset calibration to finish */
            }
          }
          break;
        }

        /* CHARGE_BOOT_CAP: 充自举电容。下桥臂常通给自举电容充电(~10ms),
         * 充电完成后切换速度源为虚拟转速传感器,清零 FOC,然后开 PWM 进入 START */
        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_1_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );

              STO_PLL_Clear(&STO_PLL_M1);

              FOC_Clear( M1 );

                Mci[M1].State = START;
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* Nothing to be done, FW waits for bootstrap capacitor to charge */
            }
          }
          break;
        }

        /* START: 开环启动。用 RevUp 流程按预设曲线强制施加电流/速度拖动电机旋转,
         * 同时等待反电势观测器(STO-PLL)收敛;收敛后进入 SWITCH_OVER 平滑过渡到闭环 */
        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* Mechanical speed as imposed by the Virtual Speed Sensor during the Rev Up phase. */
            int16_t hForcedMecSpeedUnit;
            qd_t IqdRef;
            bool ObserverConverged;

            /* 执行 RevUp 开环启动流程:按预设阶段曲线(电流/速度/加速度)强制拖动电机 */
            if(! RUC_Exec(&RevUpControlM1))
            {
            /* The time allowed for the startup sequence has expired */
            /* 启动超时未完成 -> 启动失败故障 */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
            else
            {
              /* Execute the torque open loop current start-up ramp:
               * Compute the Iq reference current as configured in the Rev Up sequence */
              /* 开环阶段: 由 STC 算出 Iq 参考(此时速度环输出受 RevUp 钳位) */
              IqdRef.q = STC_CalcTorqueReference(pSTC[M1]);
              IqdRef.d = FOCVars[M1].UserIdref;
              /* Iqd reference current used by the High Frequency Loop to generate the PWM output */
              FOCVars[M1].Iqdref = IqdRef;
           }

            (void)VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);

            /* Check that startup stage where the observer has to be used has been reached */
            /* 启动到达第一加速阶段后,转速足够,反电势可观,开始检查观测器是否收敛 */
            if (true == RUC_FirstAccelerationStageReached(&RevUpControlM1))
            {
              ObserverConverged = STO_PLL_IsObserverConverged(&STO_PLL_M1, &hForcedMecSpeedUnit);
              STO_SetDirection(&STO_PLL_M1, (int8_t)MCI_GetImposedMotorDirection(&Mci[M1]));

              (void)VSS_SetStartTransition(&VirtualSpeedSensorM1, ObserverConverged);
            }
            else
            {
              ObserverConverged = false;
            }
            if (ObserverConverged)
            {
              qd_t StatorCurrent = MCM_Park(FOCVars[M1].Ialphabeta, SPD_GetElAngle(&STO_PLL_M1._Super));

              /* Start switch over ramp. This ramp will transition from the revup to the closed loop FOC */
              REMNG_Init(pREMNG[M1]);
              (void)REMNG_ExecRamp(pREMNG[M1], FOCVars[M1].Iqdref.q, 0);
              (void)REMNG_ExecRamp(pREMNG[M1], StatorCurrent.q, TRANSITION_DURATION);

              Mci[M1].State = SWITCH_OVER;
            }
          }
          break;
        }

        /* SWITCH_OVER: 切换过渡。把转速源从虚拟传感器切换到 STO-PLL 观测器,
         * 用斜坡把 Iq 参考从开环值过渡到闭环值,过渡完成后进入 RUN 闭环运行 */
        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            int16_t hForcedMecSpeedUnit;

            /* Compute the virtual speed and positions of the rotor.
               The function returns true if the virtual speed is in the reliability range */
            /* 虚拟传感器计算转子虚拟转速/位置;返回 true 表示速度已进入可信范围 */
            bool FlagEnableClosedLoop = VSS_CalcAvrgMecSpeedUnit(&VirtualSpeedSensorM1, &hForcedMecSpeedUnit);
            /* Check if the transition ramp has completed. */
            /* 切换斜坡是否已结束(即开环->闭环的过渡时间是否走完) */
            bool FlagTransitionPhaseCompleted = VSS_TransitionEnded(&VirtualSpeedSensorM1);
            FlagEnableClosedLoop = FlagEnableClosedLoop || FlagTransitionPhaseCompleted;

            /* If any of the above conditions is true, the loop is considered closed.
               The state machine transitions to the RUN state */
            if (true == FlagEnableClosedLoop)
            {
#if PID_SPEED_INTEGRAL_INIT_DIV == 0
              /* 速度环积分项清零,避免切换瞬间残留积分造成电流冲击 */
              PID_SetIntegralTerm(&PIDSpeedHandle_M1, 0);
#else
              PID_SetIntegralTerm(&PIDSpeedHandle_M1,
                                  (((int32_t)FOCVars[M1].Iqdref.q * (int16_t)PID_GetKIDivisor(&PIDSpeedHandle_M1))
                                  / PID_SPEED_INTEGRAL_INIT_DIV));
#endif
              /* USER CODE BEGIN MediumFrequencyTask M1 1 */

              /* USER CODE END MediumFrequencyTask M1 1 */
              /* 关键: 把转速源正式切换为 STO-PLL 观测器,完成无感闭环 */
              STC_SetSpeedSensor(pSTC[M1], &STO_PLL_M1._Super); /* Observer has converged */
              FOC_InitAdditionalMethods(M1);
              FOC_CalcCurrRef(M1);
              /* 把速度参考对齐到当前实测转速,避免切换瞬间速度阶跃 */
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]); /* Init the reference speed to current speed */
              MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
              Mci[M1].State = RUN;
            }
            else if ((FlagTransitionPhaseCompleted == true) && (FlagEnableClosedLoop == false))
            {
              /* The transition time from Open-Loop to Close-Loop allowed has expired */
              /* 过渡时间已到但速度仍不可信 -> 启动失败 */
              MCI_FaultProcessing(&Mci[M1], MC_START_UP, 0);
            }
          }
          break;
        }

        /* RUN: 闭环运行。执行速度斜坡、计算 Iqdref(含弱磁/MTPA)、转速反馈检查,
         * 实际的电流环 PI 与 SVPWM 在 16kHz 高频任务中执行 */
        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */

            MCI_ExecBufferedCommands(&Mci[M1]);

              FOC_CalcCurrRef(M1);
              if(!SPD_Check((SpeednPosFdbk_Handle_t *)&STO_PLL_M1))
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                /* Nothing to do */
              }
          }
          break;
        }

        /* STOP: 停机保持。等待一段保持时间(STOPPERMANENCY_TICKS)让电机彻底停转,
         * 然后切回虚拟传感器、清零,回到 IDLE */
        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {

            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);    /* Sensor-less */
            VSS_Clear(&VirtualSpeedSensorM1); /* Reset measured speed in IDLE */
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */

            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW waits for to stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It re-initializes the current and voltage variables. Moreover
  *         it clears qd currents PI controllers, voltage sensor and SpeednTorque
  *         controller. It must be called before each motor restart.
  *         It does not clear speed sensor.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */

  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {((int16_t)0), ((int16_t)0)};
  qd_t NULL_qd = {((int16_t)0), ((int16_t)0)};
  alphabeta_t NULL_alphabeta = {((int16_t)0), ((int16_t)0)};

  FOCVars[bMotor].Iab = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
  FOCVars[bMotor].hTeref = (int16_t)0;
  FOCVars[bMotor].Vqd = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle = (int16_t)0;

  PID_SetIntegralTerm(pPIDIq[bMotor], ((int32_t)0));
  PID_SetIntegralTerm(pPIDId[bMotor], ((int32_t)0));

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  if (NULL == pFW[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FW_Clear(pFW[bMotor]);
  }
  if (NULL == pFF[bMotor])
  {
    /* Nothing to do */
  }
  else
  {
    FF_Clear(pFF[bMotor]);
  }

  MC_Perf_Clear(&PerfTraces,bMotor);
  /* USER CODE BEGIN FOC_Clear 1 */

  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
    if (M_NONE == bMotor)
    {
      /* Nothing to do */
    }
    else
    {
      if (NULL == pFF[bMotor])
      {
        /* Nothing to do */
      }
      else
      {
        FF_InitFOCAdditionalMethods(pFF[bMotor]);
      }
  /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */

  /* USER CODE END FOC_InitAdditionalMethods 0 */
    }
}

/**
  * @brief  It computes the new values of Iqdref (current references on qd
  *         reference frame) based on the required electrical torque information
  *         provided by oTSC object (internally clocked).
  *         If implemented in the derived class it executes flux weakening and/or
  *         MTPA algorithm(s). It must be called with the periodicity specified
  *         in oTSC parameters.
  * @param  bMotor related motor it can be M1 or M2.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference latching */
  /* to avoid MF task writing them while HF task reading them */
  /* 临界区保护: Iqdref 由中频(1kHz)写、高频(16kHz)读,须关中断避免竞争 */
  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;

  /* Exit critical section */
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */

  /* USER CODE END FOC_CalcCurrRef 0 */
  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    /* 内部驱动模式: 由速度环算出转矩(电流)参考
     * STC_CalcTorqueReference 内部做:速度斜坡 + 速度PI -> 转矩电流参考 Iqref */
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;

    if (NULL == pFW[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      /* 弱磁/MTPA: 在高速或需要效率优化时,重新分配 Id/Iq 参考比例 */
      IqdTmp.d = FOCVars[bMotor].UserIdref;
      IqdTmp = FW_CalcCurrRef(pFW[bMotor], IqdTmp);
    }
    if (NULL == pFF[bMotor])
    {
      /* Nothing to do */
    }
    else
    {
      /* 前馈计算: 根据当前 Iqd 与转速,提前算出电压前馈量,减小 PI 负担 */
      FF_VqdffComputation(pFF[bMotor], IqdTmp, pSTC[bMotor]);
    }
  }
  else
  {
    /* Nothing to do */
  }

  /* Enter critical section */
  /* Disable interrupts to avoid any interruption during Iqd reference restoring */
  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;

  /* Exit critical section */
  __enable_irq();
  /* USER CODE BEGIN FOC_CalcCurrRef 1 */

  /* USER CODE END FOC_CalcCurrRef 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing.
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  * @param bMotorNbr Motor reference number defined
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */

  /* 高频任务入口: 由 ADC 注入序列结束中断(JEOS)触发,频率=16kHz
   * 这里先处理常规 ADC 转换(母线电压、温度、电位器等,由 RCM 调度) */
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  Observer_Inputs_t STO_Inputs; /* Only if sensorless main */

  /* 把上一次 FOC 算出的 Vαβ 作为观测器输入(反电势观测需要定子电压) */
  STO_Inputs.Valfa_beta = FOCVars[M1].Valphabeta;  /* Only if sensorless */
  if (SWITCH_OVER == Mci[M1].State)
  {
    /* 切换过渡阶段: 用斜坡管理器把 Iq 参考从开环启动值平滑过渡到闭环值,
     * 避免无感切换瞬间电流突变造成失步 */
    if (!REMNG_RampCompleted(pREMNG[M1]))
    {
      FOCVars[M1].Iqdref.q = (int16_t)REMNG_Calc(pREMNG[M1]);
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */
  /* 调用 FOC 电流环核心:Clarke->Park->PI->Rev_Park->SVPWM */
  hFOCreturn = FOC_CurrControllerM1();
  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */
  if(hFOCreturn == MC_DURATION)
  {
    /* 占空比写入太晚,已赶不上当前 PWM 周期更新 -> 报错 */
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    bool IsAccelerationStageReached = RUC_FirstAccelerationStageReached(&RevUpControlM1);
    /* 根据状态机决定是否更新观测器(IDLE/FAULT/CHARGE_BOOT_CAP 等阶段不需要) */
    if (true == AngleSpeedEstimation[Mci[M1].State])
    {
      /* 无感观测器输入: 当前定子电流 Iαβ 与母线电压,据此估算反电势 -> 角度/速度 */
      STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta; /* Only if sensorless */
      STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(&(BusVoltageSensor_M1._Super)); /* Only for sensorless */
      (void)STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);
      STO_PLL_CalcAvrgElSpeedDpp(&STO_PLL_M1); /* Only in case of Sensor-less */
    }
    else
    {
      /* Nothing to do */
    }

    if (false == IsAccelerationStageReached)
    {
      /* 启动早期转速太低,观测器不可信 -> 复位 PLL,使用虚拟转速传感器强制拖动 */
      STO_ResetPLL(&STO_PLL_M1);
    }
    else
    {
      /* Nothing to do */
    }
    /* Only for sensor-less */
    if((START == Mci[M1].State) || (SWITCH_OVER == Mci[M1].State))
    {
      /* 把观测器角度融合进虚拟转速传感器,实现开环->闭环的平滑切换 */
      int16_t hObsAngle = SPD_GetElAngle(&STO_PLL_M1._Super);
      (void)VSS_CalcElAngle(&VirtualSpeedSensorM1, &hObsAngle);
    }
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */

    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }

  return (bMotorNbr);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. Reference frame transformations are carried out
  *        accordingly to the active speed sensor. It must be called periodically
  *        when new motor currents have been converted
  * @param this related object of class CFOC.
  * @retval int16_t It returns MC_NO_FAULTS if the FOC has been ended before
  *         next PWM Update event, MC_DURATION otherwise
  */
/* ====== FOC 控制核心函数 ======
 * 本函数是整个磁场定向控制(FOC)的心脏,在每次 ADC 注入采样完成中断中被调用。
 * 完整执行链: 读取电流 -> Clarke 变换 -> Park 变换 -> 两个 PI 电流环 ->
 *            圆限制 -> 反 Park 变换 -> SVPWM 输出
 * 执行频率 = PWM 频率 / REGULATION_EXECUTION_RATE = 16000Hz / 1 = 16kHz
 */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;

  /* 获取当前激活的转速传感器(无感时为 STO-PLL 观测器,启动阶段为虚拟转速传感器)
   * 之所以要从 STC 获取,是因为启动过程会在 VSS 与 STO 之间切换 */
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);

  /* 读取转子电角度 θ(用于后续 Park 旋转,把交流量"拉直"为直流量) */
  hElAngle = SPD_GetElAngle(speedHandle);
  /* 角度前向补偿:补偿算法执行期间的转子位置变化(本工程系数为 0,即不补偿) */
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*PARK_ANGLE_COMPENSATION_FACTOR;

  /* 步骤1: 读取电机三相相电流(实际上由 R3_1_GetPhaseCurrents 读 ADC 注入寄存器) */
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);

  /* 步骤2: Clarke 变换 —— 把三相静止坐标系 (a,b) 变换到两相静止坐标系 (α,β)
   * 公式: Iα = Ia ;  Iβ = (Ia + 2*Ib) / √3 */
  Ialphabeta = MCM_Clarke(Iab);

  /* 步骤3: Park 变换 —— 把两相静止坐标系 (α,β) 旋转到转子磁链同步坐标系 (d,q)
   * 这样 d 轴对准磁链方向、q 轴对准转矩方向,两轴电流都成为直流量,便于 PI 调节
   * 公式: Id = Iα*sinθ + Iβ*cosθ ;  Iq = Iα*cosθ - Iβ*sinθ */
  Iqd = MCM_Park(Ialphabeta, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    /* 步骤4: 两个独立的 PI 电流环(此时 Iqdref 与 Iqd 均为直流量,PI 易于调节)
     * - q 轴电流环 -> 控制电磁转矩
     * - d 轴电流环 -> 控制定子磁链(永磁同步电机一般把 Idref 设为 0,即最大转矩电流比) */
    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    /* PWM 关闭时输出电压为零(避免在充电自举电容等阶段误输出) */
    Vqd.q = 0;
    Vqd.d = 0;
  }
  /* 前馈补偿:叠加 d/q 轴电压前馈量,提升动态响应 */
  Vqd = FF_VqdConditioning(pFF[M1],Vqd);

  /* 圆限制:把 Vqd 限制在电压矢量圆内,防止总输出超过逆变器的最大可输出电压
   * (即不超调制定 SVPWM 的线性区,保证六边形/圆形内矢量幅值不越限) */
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);

  /* 反 Park 变换前再次角度补偿(系数为 0,不补偿),补偿从读角度到反变换期间的角度增量 */
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;

  /* 步骤5: 反 Park 变换 —— 把同步坐标系下的电压指令 (Vd,Vq) 还原回两相静止坐标系 (Vα,Vβ)
   * 公式: Vα = Vq*cosθ + Vd*sinθ ;  Vβ = -Vq*sinθ + Vd*cosθ */
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    /* 步骤6: SVPWM —— 根据 (Vα,Vβ) 计算扇区与三相占空比,写入 TIM1 的 CCR1/2/3
     * 同时根据扇区配置下一次 ADC 注入采样序列(决定下一次采哪两相) */
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

  }

  /* 保存本次循环的中间结果,供观测器、上位机监控、保护任务使用 */
  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  /* 弱磁数据处理(在转速高于额定、需要弱磁运行时启用) */
  FW_DataProcess(pFW[M1], Vqd);
  /* 前馈数据处理 */
  FF_DataProcess(pFF[M1]);
  return (hCodeError);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
