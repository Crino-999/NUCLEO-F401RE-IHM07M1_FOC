#ifndef __TRANSFORM_H__
#define __TRANSFORM_H__

#define sqrt_3 1.73205f
void Transform_Clarke(float ia, float ib, float ic,
                      float *i_alpha, float *i_beta);
void Transform_Park(float i_alpha, float i_beta,
                    float sin_val, float cos_val,
                    float *id, float *iq);
void Transform_InversePark(float vd, float vq,
                           float sin_val, float cos_val,
                           float *v_alpha, float *v_beta);
void Transform_InverseClarke(float v_alpha, float v_beta,
                             float *va, float *vb, float *vc);

#endif