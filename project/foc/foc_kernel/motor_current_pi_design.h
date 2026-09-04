#ifndef MOTOR_CURRENT_PI_DESIGN_H
#define MOTOR_CURRENT_PI_DESIGN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  int32_t direct_kp_q15;
  int32_t quadrature_kp_q15;
  int32_t integral_gain_q15;
} motor_current_pi_design_result_t;

/**
 * @brief 根据电机Rs/Ld/Lq和目标带宽计算10 kHz电流PI参数。
 * @param phase_resistance_mohm 单相电阻，单位mΩ且必须大于0。
 * @param direct_inductance_uh d轴电感，单位µH且必须大于0。
 * @param quadrature_inductance_uh q轴电感，单位µH且必须大于0。
 * @param bandwidth_hz 目标闭环带宽，单位Hz且必须大于0。
 * @param sample_frequency_hz 电流环更新频率，单位Hz且必须大于带宽。
 * @param result 输出d/q比例增益及公共单周期积分增益Q15，不允许为空。
 * @return 输入有效且三个结果均在int32范围内时返回true，否则返回false。
 * @details 使用Kp=L×2πfc、Ki_step=R×2πfc/fs；输入mA、输出mV时数值单位仍为V/A。
 */
bool motor_current_pi_design(uint32_t phase_resistance_mohm,
                             uint32_t direct_inductance_uh,
                             uint32_t quadrature_inductance_uh,
                             uint32_t bandwidth_hz,
                             uint32_t sample_frequency_hz,
                             motor_current_pi_design_result_t *result);

#endif
