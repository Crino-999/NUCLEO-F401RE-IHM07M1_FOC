#ifndef __SVPWM_H
#define __SVPWM_H

/* 输入：v_alpha / v_beta 单位 V，vdc 单位 V
 * 输出：da / db / dc 三相【高侧开关的导通占空比】，范围 0.0 ~ 1.0
 * 内部时间单位：一个 PWM 周期 = 1.0
 * 注：da/db/dc 为 0.5 时三相零矢量，无输出电压 */
void Svpwm_Calc(float v_alpha, float v_beta, float vdc,
                float *da, float *db, float *dc);

#endif // __SVPWM_H