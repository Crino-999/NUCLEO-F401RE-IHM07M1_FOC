#ifndef __ENCODER_H__
#define __ENCODER_H__
#include <math.h>
#include "as5600.h"

#ifndef M_PI
#define M_PI  3.14159265358979f
#endif

#define POLE_PAIRS  7 /*电机极对数*/
#define RAW_TO_DEG  (360.0f / 4096.0f)  /*角度原始值*/

void  Encoder_SetOffset(float offset_deg);   // 设置零点偏移
float Encoder_GetMechAngle(void);             // 机械角度 [0, 360)
float Encoder_GetElecAngle(void);            // 电角度 [0, 360)
float Encoder_GetElecRad(void);              // 电角度弧度 [0, 2π)
void  Encoder_GetSinCos(float *sin_val, float *cos_val);  // Park 变换用

#endif // __ENCODER_H__