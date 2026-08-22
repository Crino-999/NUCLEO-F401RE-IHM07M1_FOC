#include "transform.h"
void Transform_Clarke(float ia, float ib, float ic,
                      float *i_alpha, float *i_beta)
{
    *i_alpha = ia - (ib + ic) / 2.0f;
    *i_beta = sqrt_3 * (ib - ic) / 2.0f;
}

void Transform_Park(float i_alpha, float i_beta, float sin_val, float cos_val, float *id, float *iq)
{
    *iq = i_beta * cos_val - i_alpha * sin_val;
    *id = i_alpha * cos_val + i_beta * sin_val;
}

void Transform_InverseClarke(float v_alpha, float v_beta, float *va, float *vb, float *vc)
{
    *va = v_alpha;
    *vb = (sqrt_3 * v_beta - v_alpha)/ 2.0f;
    *vc = -0.5f * (v_alpha + sqrt_3 * v_beta);
}

void Transform_InversePark(float vd, float vq, float sin_val, float cos_val, float *v_alpha, float *v_beta)
{
    *v_alpha = vd * cos_val - vq * sin_val;
    *v_beta = vq * cos_val + vd * sin_val;
}
