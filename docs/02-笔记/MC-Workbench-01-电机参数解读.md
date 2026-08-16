# 电机参数解读：从 yuntai2804.json 到驱动代码

> 状态：待审阅 | 日期：2026-08-08 | 主题：MC-Workbench | 序号：01
> 关联：《学习总计划》阶段1；`docs/04-资料/yuntai2804.json`；`NUCLEO-F401RE/Inc/pmsm_motor_parameters.h`、`power_stage_parameters.h`、`drive_parameters.h`

## 一、学习目标

搞懂 2804 电机的每个参数是什么含义、为什么是这个值、它在 MC Workbench 界面里怎么配置、最终如何变成驱动代码里的宏定义。这是理解"配置生成代码"工具链的第一步，也是后续理解 FOC 算法的基础。

## 二、参数全景：一个值，三处呈现

同一个电机参数在工具链里出现在三个地方，理解这条"参数流水线"是掌握 MC Workbench 的关键：

```
MC Workbench 电机库 (yuntai2804.json)
        │  导入/选择电机时读取
        ▼
MC Workbench 工程配置 (.stwb6)
        │  生成代码时展开
        ▼
生成的 C 头文件 (pmsm_motor_parameters.h 等)
```

| 参数 | json 文件 | stwb6 配置 | 生成代码宏 | 含义 |
|---|---|---|---|---|
| 极对数 | `polePairs: 7` | `"polePairs": 7` | `POLE_PAIR_NUM 7` | 转子磁极对数，电角度=机械角度×7 |
| 相电阻 | `rs: 3.194` | `"rs": 3.194` | `RS 3.19` | 每相绕组电阻（Ω），决定电流与发热 |
| 相电感 | `ls: 0.8` | `"ls": 0.8` | `LS 0.0008` | 每相绕组电感（mH），决定电流响应速度 |
| 额定电流 | `nominalCurrent: 2` | `"nominalCurrent": 2` | `NOMINAL_CURRENT_A 2` | 额定电流（A），用于 PID 输出限幅 |
| 额定电压 | `nominalDCVoltage: 12.245` | `"nominalDCVoltage": 12.245` | `NOMINAL_BUS_VOLTAGE_V 12` | 额定母线电压（V） |
| 额定转速 | `maxRatedSpeed: 2502` | `"maxRatedSpeed": 2502` | `MOTOR_MAX_SPEED_RPM 2502` | 额定转速（rpm），限制最高转速 |
| 反电动势常数 | `BEmfConstant: 3.31` | `"BEmfConstant": 3.31` | `MOTOR_VOLTAGE_CONSTANT 3.3` | V RMS ph-ph / kRPM，反电动势强度 |
| 磁体结构 | `magneticStructure: SM-PMSM` | 同左 | 无直接宏 | 表贴式永磁同步电机 |

## 三、关键参数逐个解读

### 3.1 极对数 polePairs = 7（最重要）

转子有多少对磁极。**电角度 = 机械角度 × 极对数**。你的电机转一圈（360°机械角），电角度转 7 圈（7×360°）。FOC 控制的所有计算都发生在电气域，所以极对数错了，整个控制就是错的。这也是为什么"极对数"是电机参数里第一个要核对的值。

### 3.2 相电阻 RS = 3.19 Ω

每相绕组（电机三相中的一相）的电阻。它决定：
- 相同电压下能产生多大电流（I = V/R）
- 铜损发热（P = I²R）
- 反电动势估算（无感 FOC 需要用它做观测器模型）

3.19Ω 对云台电机来说是正常量级——云台电机通常额定电压低（12V 级）、电流小（2A 级）。

### 3.3 相电感 LS = 0.8 mH

每相绕组电感。它决定电流变化的快慢：电感越小，电流响应越快（dI/dt = V/L）。对电流环 PI 参数的整定很关键——电感和电阻构成电气时间常数 τ = L/R = 0.8mH / 3.19Ω ≈ 0.25ms，决定了电流环带宽上限。

### 3.4 反电动势常数 BEmfConstant = 3.31

电机转动时绕组切割磁力线产生的反电动势强度，单位 V RMS ph-ph / kRPM。含义：转速 1000 rpm 时，线间反电动势 RMS 值为 3.31V。它决定：
- 最高转速受母线电压限制（反电动势不能超过母线电压）
- 无感 FOC 观测器靠它估算转子位置和速度
- 弱磁控制的起点

### 3.5 额定电流/电压/转速

- **额定电流 2A**：电机安全运行的电流上限。生成代码里 `IQMAX_A 2` 限制 q 轴电流指令，`NOMINAL_CURRENT_A 2` 用于速度环输出限幅
- **额定电压 12.245V**：与你的 12V 供电一致（母线电压，可调电源 12V）
- **额定转速 2502rpm**：`MAX_APPLICATION_SPEED_RPM 2502` 限制应用最高转速

## 四、参数如何落地为代码（生成链验证）

### 4.1 电流环与速度环参数（drive_parameters.h）

| 宏 | 值 | 对应界面/含义 |
|---|---|---|
| `PWM_FREQUENCY` | 16000 | PWM 开关频率 16kHz |
| `ISR_FREQUENCY_HZ` | 16000 | FOC 控制频率 = PWM 频率（每 PWM 周期执行一次） |
| `PID_TORQUE_KP_DEFAULT` | 2627 | 电流环（转矩环）Kp，定点数表示 |
| `PID_SPEED_KP_DEFAULT` | 2881/(SPEED_UNIT/10) | 速度环 Kp，按 0.1Hz 单位标定 |
| `IQMAX_A` | 2 | q 轴电流上限 = 额定电流 2A |
| `SPEED_LOOP_FREQUENCY_HZ` | 1000 | 速度环执行频率 1kHz |

### 4.2 无传感器 FOC 的证据（重要发现）

阅读 `drive_parameters.h` 发现：**你生成的工程是无传感器 FOC（Sensorless），不是基于 AS5600 编码器的有感方案**。证据：

- `drive_parameters.h` 里 `STARTING_ANGLE_DEG`、`OBS_MINIMUM_SPEED_RPM 901`、`NB_CONSECUTIVE_TESTS`、`SPEED_BAND_*`、`TRANSITION_DURATION` —— 这是典型的"开环强拖 → 切换到闭环观测器"启动序列，无感 FOC 专属
- 有状态观测器（State Observer）参数 `GAIN1`、`GAIN2`、`F1`、`F2` 和 PLL 参数 `PLL_KP_GAIN`、`PLL_KI_GAIN` —— 用反电动势观测器 + PLL 估算转子位置/速度
- 对比：SimpleFOC 分支（正点原子板）用的是 AS5600 磁编码器实测角度，两者定位原理完全不同

这个发现解释了一个现象：IHM07M1 板上没有霍尔/编码器输入（它是低电压驱动板，靠电流采样做无感控制），所以 MC Workbench 默认给你生成无感方案。**理解"无感"与"有感"的区别，是后续学习的重点**。

## 五、与正点原子板方案的对比

| 维度 | NUCLEO 方案（本仓库） | SimpleFOC 方案（旧分支） |
|---|---|---|
| 位置传感器 | 无（反电动势观测器+PLL） | AS5600 磁编码器 |
| 控制算法 | ST MCSDK 定点 FOC | SimpleFOC 浮点 FOC |
| 电机参数来源 | MC Workbench 电机库（json） | 代码手动配置 |
| 工具链 | MC Workbench 生成 | 手写代码 |
| 启动方式 | 开环强拖 → 闭环过渡 | 开环对齐 → 闭环 |

## 六、遗留疑问（下次学习待解决）

1. MC Workbench 界面里"电流采样配置"（shunt 电阻 0.33Ω、放大增益 1.53）与 `power_stage_parameters.h` 的对应关系，需要打开界面对照
2. 定点数 PID 参数（2627、2881 等）是怎么由浮点 Kp 换算出来的（`parameters_conversion_f4xx.h` 应该有换算逻辑）
3. 无感 FOC 的开环启动序列（Phase1-5）具体每个阶段在干什么

## 七、本次收获

1. 建立了"电机参数 → 界面配置 → 生成代码"的完整映射关系，MC Workbench 不再是黑盒
2. 发现工程是无感 FOC 方案，明确了"反电动势观测器"是后续学习的核心
3. 验证了 `yuntai2804.json` 与生成代码的一致性（polePairs/rs/ls/nominalCurrent 等完全对应）
4. 理解了电机参数之间的物理关系（τ=L/R、反电动势与转速的关系）
