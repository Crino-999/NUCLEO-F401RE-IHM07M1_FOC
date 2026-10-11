#include "svpwm.h"


#define SQRT3 1.73205080757f

void Svpwm_Calc(float v_alpha, float v_beta, float vdc,
                float Ts, uint32_t ARR,
                uint32_t *ccr_a, uint32_t *ccr_b, uint32_t *ccr_c)
{
    /* 1. 计算 X, Y, Z */
    float X = v_beta;
    float Y = (SQRT3 / 2.0f) * v_alpha - 0.5f * v_beta;
    float Z = -(SQRT3 / 2.0f) * v_alpha - 0.5f * v_beta;

    /* 2. 扇区判断 */
    int A = (X > 0.0f) ? 1 : 0;
    int B = (Y > 0.0f) ? 1 : 0;
    int C = (Z > 0.0f) ? 1 : 0;
    int N = 4 * C + 2 * B + A;

    int sector = 0;
    switch (N) {
        case 3: sector = 1; break;
        case 1: sector = 2; break;
        case 5: sector = 3; break;
        case 4: sector = 4; break;
        case 6: sector = 5; break;
        case 2: sector = 6; break;
        default: sector = 0; break;
    }

    /* 3. 计算 TX, TY */
    float K = SQRT3 * Ts / vdc;
    float TX = 0.0f;
    float TY = 0.0f;

    switch (sector) {
        case 1: TX =  K * Y;  TY =  K * X;  break;
        case 2: TX = -K * Y;  TY = -K * Z;  break;
        case 3: TX =  K * X;  TY =  K * Z;  break;
        case 4: TX = -K * X;  TY = -K * Y;  break;
        case 5: TX =  K * Z;  TY =  K * Y;  break;
        case 6: TX = -K * Z;  TY = -K * X;  break;
    }

    /* 4. 过调制限幅 */
    float sum = TX + TY;
    if (sum > Ts) {
        float k = Ts / sum;
        TX *= k;
        TY *= k;
    }
    float T0 = Ts - TX - TY;

    /* 5. 三个比较点（归一化到 [0, Ts]） */
    float Ta_prime = T0 * 0.25f;
    float Tb_prime = Ta_prime + TX * 0.5f;
    float Tc_prime = Tb_prime + TY * 0.5f;

    float CmpA = 0.0f, CmpB = 0.0f, CmpC = 0.0f;

    switch (sector) {
        case 1: CmpA = Ta_prime; CmpB = Tb_prime; CmpC = Tc_prime; break;
        case 2: CmpA = Tb_prime; CmpB = Ta_prime; CmpC = Tc_prime; break;
        case 3: CmpA = Tc_prime; CmpB = Ta_prime; CmpC = Tb_prime; break;
        case 4: CmpA = Tc_prime; CmpB = Tb_prime; CmpC = Ta_prime; break;
        case 5: CmpA = Tb_prime; CmpB = Tc_prime; CmpC = Ta_prime; break;
        case 6: CmpA = Ta_prime; CmpB = Tc_prime; CmpC = Tb_prime; break;
        default: CmpA = CmpB = CmpC = 0.5f; break;   /* 零矢量，三相全低 */
    }

    /* 6. 直接输出 CCR */
    uint32_t CCR_A = (uint32_t)((Ts - 2*CmpA) * (float)ARR / Ts + 0.5f);
    uint32_t CCR_B = (uint32_t)((Ts - 2*CmpB) * (float)ARR / Ts + 0.5f);
    uint32_t CCR_C = (uint32_t)((Ts - 2*CmpC) * (float)ARR / Ts + 0.5f);

    /* 防越界 */
    if (CCR_A > ARR) CCR_A = ARR;
    if (CCR_B > ARR) CCR_B = ARR;
    if (CCR_C > ARR) CCR_C = ARR;

    *ccr_a = CCR_A;
    *ccr_b = CCR_B;
    *ccr_c = CCR_C;
}