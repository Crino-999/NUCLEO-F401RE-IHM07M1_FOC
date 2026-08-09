# MC-Workbench-02：生成工程代码结构解读

> 状态：待审阅 | 日期：2026-08-09 | 主题：MC-Workbench | 序号：02
> 关联：《学习总计划》阶段1；`NUCLEO-F401RE/Src/`、`MCSDK_v6.4.2-Full/MotorControl/MCSDK/MCLib/`

## 一、总体架构：三层结构

工程是 ST Motor Control SDK（MCSDK）标准组织方式：

```
NUCLEO-F401RE/
├── Src/                  # 应用层：ST 生成的任务调度、状态机、接口
├── Inc/                  # 应用层头文件 + 参数头文件
└── MCSDK_v6.4.2-Full/
    └── MotorControl/MCSDK/MCLib/Any/
        ├── Src/          # 算法库：PID、观测器、坐标变换、弱磁等
        └── Inc/          # 算法库头文件
```

核心认知：**`main()` 主循环为空，电机控制完全由中断驱动**。FOC 电流环在中断里（16kHz），速度环在定时调度里（1kHz），主循环留给用户代码。

## 二、启动流程：main()

```
main()
├── HAL_Init()
├── SystemClock_Config()          # HSE 8MHz → PLL → 84MHz
├── MX_GPIO_Init()                # 三相 PWM 使能、启动按钮
├── MX_DMA_Init()
├── MX_ADC1_Init()                # 注入通道：三相电流（T1_CC4 触发）+ 母线电压/温度
├── MX_TIM1_Init()                # 中心对齐 PWM，16kHz
├── MX_USART2_UART_Init()         # 921600bps（MC Workbench 调试通道）
├── MX_MotorControl_Init()        # ★ 电机控制初始化
├── MX_NVIC_Init()
└── while(1) {}                   # 空循环
```

关键点：
- ADC 注入通道由 TIM1 CC4 事件触发采样，电流采样与 PWM 周期精确同步（中点采样避开关噪声）
- TIM1 中心对齐模式，`PWM_PERIOD_CYCLES` 决定 16kHz
- USART2 921600 是 MC Workbench 上位机通信通道

## 三、电机控制初始化：MCboot → FOC_Init

`MX_MotorControl_Init()` → `MCboot()`（`mc_tasks.c`）→ `FOC_Init()`（`mc_tasks_foc.c`），实例化所有算法组件：

| 组件 | 函数 | 作用 |
|---|---|---|
| PWM+电流采样 | `R3_1_Init` | 三相 PWM 输出 + 电流检测 |
| 速度环 PID | `PID_HandleInit(&PIDSpeedHandle_M1)` | 速度 PI 控制器 |
| 无感观测器 | `STO_PLL_Init` | 反电动势观测器+PLL，估算转子角度/速度 |
| 速度转矩控制器 | `STC_Init` | 速度→转矩参考 |
| 启动控制 | `RUC_Init` | Rev-Up 开环启动序列 |
| 电流环 PID | `PID_HandleInit(&PIDIq/IdHandle)` | d/q 轴电流 PI |
| 弱磁 | `FW_Init` | 高速弱磁 |
| 前馈 | `FF_Init` | 电流环前馈解耦 |

## 四、两级任务调度

### 高频任务：FOC 电流环（16kHz）

```
TIM1 PWM → CC4 触发 ADC 注入采样 → ADC 中断
  → TSK_HighFrequencyTask → FOC_HighFrequencyTask → FOC_CurrControllerM1
```

`FOC_CurrControllerM1()` 是 FOC 完整实现：

```c
hElAngle = SPD_GetElAngle(speedHandle);          // 1. 电角度（无感：观测器估算）
PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);     // 2. 读三相电流
Ialphabeta = MCM_Clarke(Iab);                    // 3. Clarke: abc → αβ
Iqd = MCM_Park(Ialphabeta, hElAngle);            // 4. Park: αβ → dq
Vqd.q = PI_Controller(pPIDIq, Iqref - Iqd.q);    // 5. 电流环 PI（q/d 轴）
Vqd.d = PI_Controller(pPIDId, Idref - Iqd.d);
Vqd = FF_VqdConditioning(pFF[M1], Vqd);          // 6. 前馈解耦
Vqd = Circle_Limitation(&CircleLimitationM1, Vqd); // 7. 电压圆限制
Valphabeta = MCM_Rev_Park(Vqd, hElAngle);        // 8. 反Park: dq → αβ
PWMC_SetPhaseVoltage(pwmcHandle, Valphabeta);    // 9. SVPWM → PWM 输出
```

电流环之后更新观测器（无感核心）：
```c
STO_Inputs.Ialfa_beta = FOCVars[M1].Ialphabeta;
STO_Inputs.Vbus = VBS_GetAvBusVoltage_d(...);
STO_PLL_CalcElAngle(&STO_PLL_M1, &STO_Inputs);   // 反电动势观测器+PLL → 估算电角度
```

### 中频任务：速度环 + 状态机（1kHz）

```
SysTick (1ms) → MC_RunMotorControlTasks → TSK_MediumFrequencyTaskM1 + TSK_SafetyTask
```

状态机（无感 FOC 启动灵魂）：

```
IDLE → OFFSET_CALIB → CHARGE_BOOT_CAP → START → SWITCH_OVER → RUN
```

| 状态 | 动作 |
|---|---|
| IDLE | 等待启动命令 |
| OFFSET_CALIB | 电流采样零点校准 |
| CHARGE_BOOT_CAP | 自举电容充电（上桥驱动悬浮电源） |
| START | `RUC_Exec` 开环强拖，观测器后台学习反电动势 |
| SWITCH_OVER | 观测器收敛，开环→闭环斜坡切换（`REMNG_Calc`） |
| RUN | 速度环+电流环闭环；`FOC_CalcCurrRef` 每 1ms 算 Iq 参考 |

RUN 状态速度环：
```c
MCI_ExecBufferedCommands(&Mci[M1]);   // 处理上位机命令
FOC_CalcCurrRef(M1);                  // 速度环：目标 vs 估算 → Iq 参考
SPD_Check(&STO_PLL_M1);               // 速度反馈校验
```

### 安全任务

`TSK_SafetyTask` 每 1ms：母线过压/欠压（14.7V/8.0V）、过温（110°C）、硬件过流（TIM1 BRK 中断）。任何故障 → 关 PWM + 进 FAULT。

## 五、Workbench 配置与代码对应

| 界面配置 | 代码位置 |
|---|---|
| PWM 频率 / FOC 执行率 | `PWM_FREQUENCY`、`ISR_FREQUENCY_HZ` |
| 速度环频率 | `SPEED_LOOP_FREQUENCY_HZ` |
| PID 参数 | `PID_TORQUE_*`、`PID_SPEED_*`（定点数） |
| 启动序列 | `PHASE*_DURATION` → `RUC_Exec` |
| 过压/欠压阈值 | `OV_VOLTAGE_THRESHOLD_V`、`UD_VOLTAGE_THRESHOLD_V` |
| 电机参数 | `pmsm_motor_parameters.h` |

## 六、理解要点

1. 中断分层：16kHz 电流环（ADC 中断）+ 1kHz 速度环/状态机（SysTick）
2. FOC 五步：Clarke → Park → 电流环 PI → 反 Park → SVPWM，全在 `FOC_CurrControllerM1`
3. 无感方案：转子角度由观测器+PLL 估算，无需编码器
4. 状态机启动：校准 → 充电 → 开环强拖 → 切闭环

## 七、遗留疑问

1. `parameters_conversion.h` 中定点数与浮点数的换算逻辑（PID 增益 2627 等如何从浮点 Kp 得出）
2. SVPWM 实现 `PWMC_SetPhaseVoltage` 在库中如何工作（`pwm_curr_fdbk_ovm.c`）
3. 电流采样 `PWMC_GetPhaseCurrents` 三电阻采样的细节（与 `THREE_SHUNT` 宏的对应）
