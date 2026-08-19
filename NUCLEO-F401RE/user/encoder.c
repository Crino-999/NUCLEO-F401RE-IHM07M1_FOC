#include "encoder.h"

static float offset = 0.0f;  // 零点偏移值 (度)
void  Encoder_SetOffset(float offset_deg)
{
    offset = offset_deg;
}

float Encoder_GetMechAngle(void)
{
    uint16_t raw = AS5600_ReadRawAngle();
    return ((float)raw * RAW_TO_DEG);
}

float Encoder_GetElecAngle(void)
{
    float mech_angle = Encoder_GetMechAngle();
    float elec_angle = fmodf(((mech_angle - offset) * POLE_PAIRS), 360.0f);
    if (elec_angle < 0.0f)
        elec_angle += 360.0f;
    return elec_angle;
}

float Encoder_GetElecRad(void)
{
    float elec_angle = Encoder_GetElecAngle();
    return (elec_angle * (M_PI / 180.0f));  // 转换为弧度
}

void  Encoder_GetSinCos(float *sin_val, float *cos_val)
{
    float elec_rad = Encoder_GetElecRad();
    *sin_val = sinf(elec_rad);
    *cos_val = cosf(elec_rad);
}
