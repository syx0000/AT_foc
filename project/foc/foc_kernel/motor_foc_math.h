#ifndef MOTOR_FOC_MATH_H
#define MOTOR_FOC_MATH_H

#include <stdint.h>

typedef struct
{
  int16_t sin_q15;
  int16_t cos_q15;
} motor_sin_cos_q15_t;

typedef struct
{
  int16_t alpha_q15;
  int16_t beta_q15;
} motor_alpha_beta_q15_t;

typedef struct
{
  int16_t direct_q15;
  int16_t quadrature_q15;
} motor_direct_quadrature_q15_t;

typedef struct
{
  uint16_t phase_a;
  uint16_t phase_b;
  uint16_t phase_c;
} motor_svpwm_duty_q15_t;

/**
 * @brief 根据无符号一周电角度计算Q15正弦和余弦。
 * @param electrical_angle_u16 电角度，0..65535线性对应0..360度，数值自然回绕。
 * @return Q15正弦和余弦；+1使用32767表示，-1使用-32767表示。
 * @details 使用四分之一周期查表和线性插值，不使用浮点运算。
 */
motor_sin_cos_q15_t motor_foc_sin_cos_q15(uint16_t electrical_angle_u16);

/**
 * @brief 将旋转坐标系dq电压反变换为静止坐标系alpha-beta电压。
 * @param voltage_d_q15 d轴电压，Q15标幺值，基值为直流母线电压。
 * @param voltage_q_q15 q轴电压，Q15标幺值，基值为直流母线电压。
 * @param electrical_angle_u16 电角度，0..65535对应0..360度。
 * @return alpha-beta电压Q15标幺值。
 * @details 计算alpha=d*cos-q*sin，beta=d*sin+q*cos；中间结果使用32位有符号数。
 */
motor_alpha_beta_q15_t motor_foc_inverse_park_q15(int16_t voltage_d_q15,
                                                  int16_t voltage_q_q15,
                                                  uint16_t electrical_angle_u16);

/**
 * @brief 将A、B两相电流执行幅值不变Clarke变换。
 * @param phase_a_q15 A相电流Q15标幺值。
 * @param phase_b_q15 B相电流Q15标幺值。
 * @return alpha-beta电流Q15，其中alpha=Ia，beta=(Ia+2Ib)/sqrt(3)。
 */
motor_alpha_beta_q15_t motor_foc_clarke_q15(int16_t phase_a_q15,
                                             int16_t phase_b_q15);

/**
 * @brief 将静止坐标系alpha-beta量变换到旋转坐标系dq。
 * @param alpha_beta alpha-beta输入Q15标幺值。
 * @param electrical_angle_u16 电角度，0..65535对应0..360度。
 * @return dq输出Q15，其中d=alpha*cos+beta*sin，q=-alpha*sin+beta*cos。
 */
motor_direct_quadrature_q15_t motor_foc_park_q15(
  motor_alpha_beta_q15_t alpha_beta,
  uint16_t electrical_angle_u16);

/**
 * @brief 根据alpha-beta电压生成三相SVPWM占空比。
 * @param voltage alpha-beta电压Q15标幺值，基值为直流母线电压。
 * @return A/B/C三相占空比，0..32767对应0..100%。
 * @details 先完成逆Clarke变换，再注入-(最大相电压+最小相电压)/2的零序分量。
 *          输出自动限制在0..32767；调用者应将电压矢量限制在线性调制区内。
 */
motor_svpwm_duty_q15_t motor_foc_svpwm_q15(motor_alpha_beta_q15_t voltage);

#endif /* MOTOR_FOC_MATH_H */
