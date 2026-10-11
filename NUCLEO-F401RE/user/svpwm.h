#ifndef __SVPWM_H
#define __SVPWM_H
#include <stdint.h>


void Svpwm_Calc(float v_alpha, float v_beta, float vdc,
                float Ts, uint32_t ARR,
                uint32_t *ccr_a, uint32_t *ccr_b, uint32_t *ccr_c);   /* 输出CCR */


#endif // __SVPWM_H