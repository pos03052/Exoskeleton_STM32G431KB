#ifndef __TORQUE_LUT_H__
#define __TORQUE_LUT_H__

#include "main.h"

#define NUM_ROWS         50
#define NUM_COLS         50

#ifndef M_2PI
  #define M_2PI              6.283185307179586f
#endif

#ifndef M_PI
  #define M_PI               3.141592653589793f
#endif

#ifndef M_PI_2
  #define M_PI_2             1.570796326794897f
#endif

#ifndef M_PI_4
  #define M_PI_4             0.785398163397448f
#endif

#ifndef M_PI_3
  #define M_PI_3             1.047197551196598f
#endif

#ifndef M_PI_6
  #define M_PI_6             0.523598775598299f
#endif

//extern arm_bilinear_interp_instance_f32 S_trq1;
//extern arm_bilinear_interp_instance_f32 S_trq2;

//extern double rad2deg(double _rad);
//extern double deg2rad(double _deg);

void getJointTorque(const float32_t theta_rad[], float32_t torque_Nm[]);

/* TEST CODE
float32_t theta[2] = {0,};
float32_t torque[2] = {0,};

void loop_async(void)
{
  // theta[0] = M_PI_3*(1-arm_cos_f32(M_2PI*0.25*HAL_GetTick()*0.001));
  theta[0] = 0;
  theta[1] = M_PI_3*(1-arm_cos_f32(M_2PI*0.25*HAL_GetTick()*0.001));
  getJointTorque(theta, torque);
    
  // duration mean value : 10 us
}
*/

#endif /* __TORQUE_LUT_H__ */