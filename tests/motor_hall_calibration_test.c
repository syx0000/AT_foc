#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_hall_calibration.h"

static const motor_hall_calibration_config_t test_config =
{
  2U,     /* 每个方向、每个边界至少2个样本。 */
  8000U,  /* 扇区最小约44度。 */
  14000U, /* 扇区最大约77度。 */
  200U,   /* 同一边界最大离散约1.1度。 */
  300U,   /* 正反边界最大差约1.65度。 */
  1000U   /* 最大事件间隔。 */
};

/**
 * @brief 按HA/HB/HC位排列重新映射一个Hall状态。
 * @param state 原始三位Hall状态。
 * @param bit_order 新bit0/bit1/bit2对应的原始位编号。
 * @return 重新排列后的三位状态。
 */
static uint8_t state_permute(uint8_t state, const uint8_t bit_order[3])
{
  return (uint8_t)((((state >> bit_order[0]) & 1U) << 0U) |
                   (((state >> bit_order[1]) & 1U) << 1U) |
                   (((state >> bit_order[2]) & 1U) << 2U));
}

/**
 * @brief 对一种相序方向、起始扇区和Hall位排列执行完整正反标定。
 * @param reverse_sequence false使用基准顺序，true使用基准逆序。
 * @param rotation 六步序列循环起点，范围0..5。
 * @param bit_order 三路Hall位排列。
 * @return 无；失败时由assert终止测试。
 */
static void complete_calibration_test(bool reverse_sequence,
                                      uint8_t rotation,
                                      const uint8_t bit_order[3])
{
  static const uint8_t base_sequence[6] = {1U, 5U, 4U, 6U, 2U, 3U};
  motor_hall_calibration_t calibration;
  motor_hall_calibration_result_t result;
  uint8_t sequence[6];
  uint16_t entry_angle[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint8_t index;
  uint8_t source_index;
  uint8_t lap;
  uint8_t previous;
  uint8_t current;
  uint32_t timestamp = 0U;

  for (index = 0U; index < 6U; index++)
  {
    source_index = reverse_sequence ?
      (uint8_t)((rotation + 6U - index) % 6U) :
      (uint8_t)((rotation + index) % 6U);
    sequence[index] = state_permute(base_sequence[source_index], bit_order);
    entry_angle[sequence[index]] = (uint16_t)(65000U +
      ((uint32_t)index * 10923U));
  }

  assert(motor_hall_calibration_init(&calibration, &test_config) ==
         MOTOR_HALL_CALIBRATION_OK);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) ==
         MOTOR_HALL_CALIBRATION_OK);
  for (lap = 0U; lap < 3U; lap++)
  {
    for (index = 0U; index < 6U; index++)
    {
      previous = sequence[index];
      current = sequence[(index + 1U) % 6U];
      timestamp += 100U;
      assert(motor_hall_calibration_edge_add(
        &calibration, previous, current,
        (uint16_t)(entry_angle[current] + (int16_t)lap - 1),
        timestamp) == MOTOR_HALL_CALIBRATION_OK);
    }
  }
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_OK);
  assert(motor_hall_calibration_scan_begin(&calibration, -1) ==
         MOTOR_HALL_CALIBRATION_OK);
  for (lap = 0U; lap < 3U; lap++)
  {
    for (index = 0U; index < 6U; index++)
    {
      previous = sequence[(6U - index) % 6U];
      current = sequence[(5U - index) % 6U];
      timestamp += 100U;
      assert(motor_hall_calibration_edge_add(
        &calibration, previous, current,
        (uint16_t)(entry_angle[previous] + (int16_t)lap - 1),
        timestamp) == MOTOR_HALL_CALIBRATION_OK);
    }
  }
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_OK);
  assert(motor_hall_calibration_result_get(&calibration, &result));
  assert(result.valid && result.reverse_verified);
  assert(result.maximum_edge_deviation_u16 <= 1U);
  assert(result.maximum_forward_reverse_error_u16 == 0U);
  for (index = 0U; index < 6U; index++)
  {
    assert(result.positive_next[sequence[index]] ==
           sequence[(index + 1U) % 6U]);
    assert(result.entry_angle_u16[sequence[index]] ==
           entry_angle[sequence[index]]);
    assert(result.forward_samples[sequence[index]] == 3U);
    assert(result.reverse_samples[sequence[index]] == 3U);
  }
}

/**
 * @brief 覆盖六种相序组合与六种Hall信号位排列。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void permutation_matrix_test(void)
{
  static const uint8_t bit_orders[6][3] =
  {
    {0U, 1U, 2U}, {0U, 2U, 1U}, {1U, 0U, 2U},
    {1U, 2U, 0U}, {2U, 0U, 1U}, {2U, 1U, 0U}
  };
  uint8_t hall_permutation;
  uint8_t phase_permutation;

  for (phase_permutation = 0U; phase_permutation < 6U;
       phase_permutation++)
  {
    for (hall_permutation = 0U; hall_permutation < 6U;
         hall_permutation++)
    {
      complete_calibration_test((phase_permutation & 1U) != 0U,
                                (uint8_t)(phase_permutation / 2U),
                                bit_orders[hall_permutation]);
    }
  }
}

/**
 * @brief 验证非法状态、重复、跨状态、多后继和事件超时均锁存失败。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void transition_error_test(void)
{
  motor_hall_calibration_t calibration;

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 0U, 1U, 0U, 1U) ==
         MOTOR_HALL_CALIBRATION_INVALID_STATE);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 1U, 1U, 0U, 1U) ==
         MOTOR_HALL_CALIBRATION_DUPLICATE_STATE);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 1U, 6U, 0U, 1U) ==
         MOTOR_HALL_CALIBRATION_NON_ADJACENT_STATE);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 1U, 5U, 0U, 1U) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 1U, 3U, 0U, 2U) ==
         MOTOR_HALL_CALIBRATION_MULTIPLE_SUCCESSOR);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 1U, 5U, 0U, 1U) == 0);
  assert(motor_hall_calibration_edge_add(&calibration, 5U, 4U, 0U, 1002U) ==
         MOTOR_HALL_CALIBRATION_TIMESTAMP_INVALID);
}

/**
 * @brief 验证缺状态和DWT时间戳自然回绕行为。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void incomplete_and_wrap_test(void)
{
  motor_hall_calibration_t calibration;

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  assert(motor_hall_calibration_edge_add(
    &calibration, 1U, 5U, 1000U, 0xFFFFFFF0U) == 0);
  assert(motor_hall_calibration_edge_add(
    &calibration, 5U, 4U, 2000U, 0x00000020U) == 0);
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_INCOMPLETE_SEQUENCE);
}

/**
 * @brief 用基准六步顺序填充两圈正向样本。
 * @param calibration 已开始正向扫描的上下文。
 * @param distorted_state 需要注入第二圈角度偏差的边界状态；0表示不注入。
 * @param distortion_u16 第二圈附加角度偏差。
 * @return 最后一个事件时间戳。
 */
static uint32_t standard_forward_fill(motor_hall_calibration_t *calibration,
                                      uint8_t distorted_state,
                                      uint16_t distortion_u16)
{
  static const uint8_t sequence[6] = {1U, 5U, 4U, 6U, 2U, 3U};
  uint8_t lap;
  uint8_t index;
  uint8_t current;
  uint16_t angle;
  uint32_t timestamp = 0U;

  for (lap = 0U; lap < 2U; lap++)
  {
    for (index = 0U; index < 6U; index++)
    {
      current = sequence[(index + 1U) % 6U];
      angle = (uint16_t)(1000U +
        ((uint32_t)((index + 1U) % 6U) * 10923U));
      if ((lap == 1U) && (current == distorted_state))
        angle = (uint16_t)(angle + distortion_u16);
      timestamp += 100U;
      assert(motor_hall_calibration_edge_add(
        calibration, sequence[index], current, angle, timestamp) == 0);
    }
  }
  return timestamp;
}

/**
 * @brief 验证边沿离散、扇区宽度和反向顺序错误会返回明确失败码。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void angle_and_reverse_error_test(void)
{
  motor_hall_calibration_t calibration;
  motor_hall_calibration_config_t sector_config = test_config;

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  (void)standard_forward_fill(&calibration, 5U, 500U);
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_EDGE_DEVIATION_EXCEEDED);

  sector_config.minimum_sector_width_u16 = 10500U;
  sector_config.maximum_sector_width_u16 = 11500U;
  assert(motor_hall_calibration_init(&calibration, &sector_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  (void)standard_forward_fill(&calibration, 5U, 1000U);
  calibration.config.maximum_edge_deviation_u16 = 1000U;
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_SECTOR_ANGLE_INVALID);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  (void)standard_forward_fill(&calibration, 0U, 0U);
  assert(motor_hall_calibration_scan_end(&calibration) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, -1) == 0);
  assert(motor_hall_calibration_edge_add(
    &calibration, 1U, 5U, 1000U, 100U) ==
    MOTOR_HALL_CALIBRATION_REVERSE_MISMATCH);

  assert(motor_hall_calibration_init(&calibration, &test_config) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, 1) == 0);
  (void)standard_forward_fill(&calibration, 0U, 0U);
  assert(motor_hall_calibration_scan_end(&calibration) == 0);
  assert(motor_hall_calibration_scan_begin(&calibration, -1) == 0);
  {
    static const uint8_t sequence[6] = {1U, 5U, 4U, 6U, 2U, 3U};
    uint8_t lap;
    uint8_t index;
    uint8_t previous;
    uint8_t current;
    uint16_t angle;
    uint32_t timestamp = 0U;

    for (lap = 0U; lap < 2U; lap++)
    {
      for (index = 0U; index < 6U; index++)
      {
        previous = sequence[(6U - index) % 6U];
        current = sequence[(5U - index) % 6U];
        angle = (uint16_t)(1000U +
          ((uint32_t)((6U - index) % 6U) * 10923U));
        if (previous == 1U) angle = (uint16_t)(angle + 500U);
        timestamp += 100U;
        assert(motor_hall_calibration_edge_add(
          &calibration, previous, current, angle, timestamp) == 0);
      }
    }
  }
  assert(motor_hall_calibration_scan_end(&calibration) ==
         MOTOR_HALL_CALIBRATION_REVERSE_MISMATCH);
}

int main(void)
{
  permutation_matrix_test();
  transition_error_test();
  incomplete_and_wrap_test();
  angle_and_reverse_error_test();
  printf("motor_hall_calibration_test: PASS\n");
  return 0;
}
