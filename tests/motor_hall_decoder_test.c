#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_hall_decoder.h"

static const uint8_t positive_next[8] =
{
  0U, 5U, 3U, 1U, 6U, 4U, 2U, 0U
};

/**
 * @brief 按三路输入位排列重新映射一个Hall组合状态。
 * @param state 原始三位Hall状态。
 * @param bit_order 新bit0/bit1/bit2分别取自的原始位编号。
 * @return 映射后的三位Hall状态。
 */
static uint8_t state_permute(uint8_t state, const uint8_t bit_order[3])
{
  return (uint8_t)((((state >> bit_order[0]) & 1U) << 0U) |
                   (((state >> bit_order[1]) & 1U) << 1U) |
                   (((state >> bit_order[2]) & 1U) << 2U));
}

/**
 * @brief 读取解码器输出并断言接口成功。
 * @param 无。
 * @return 当前完整Hall解码快照。
 */
static motor_hall_sample_t sample_get(void)
{
  motor_hall_sample_t sample;

  assert(motor_hall_decoder_sample_read(&sample));
  return sample;
}

/**
 * @brief 验证从六种合法初始状态开始均可完成正向闭环及滑动测速。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void positive_rotation_test(void)
{
  uint8_t initial_state;
  uint8_t state;
  uint8_t edge;
  uint32_t timestamp;
  motor_hall_sample_t sample;

  for (initial_state = 1U; initial_state <= 6U; initial_state++)
  {
    assert(motor_hall_decoder_init(positive_next, 6000U,
                                   initial_state, 0U));
    state = initial_state;
    timestamp = 0U;
    for (edge = 1U; edge <= 7U; edge++)
    {
      state = positive_next[state];
      timestamp += 100U;
      assert(motor_hall_decoder_edge_process(state, edge, timestamp));
    }
    sample = sample_get();
    assert(sample.direction == 1);
    assert(sample.positive_count == 7U);
    assert(sample.negative_count == 0U);
    assert(sample.electrical_frequency_millihz == 10000U);
    assert(sample.frequency_update_count == 6U);
  }
}

/**
 * @brief 验证HA/HB/HC六种接线排列生成的运行时表均可正确解码。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void hall_permutation_test(void)
{
  static const uint8_t bit_orders[6][3] =
  {
    {0U, 1U, 2U}, {0U, 2U, 1U}, {1U, 0U, 2U},
    {1U, 2U, 0U}, {2U, 0U, 1U}, {2U, 1U, 0U}
  };
  uint8_t mapped_next[8];
  uint8_t permutation;
  uint8_t state;
  uint8_t mapped_state;
  uint8_t edge;
  motor_hall_sample_t sample;

  for (permutation = 0U; permutation < 6U; permutation++)
  {
    for (state = 0U; state < 8U; state++) mapped_next[state] = 0U;
    for (state = 1U; state <= 6U; state++)
    {
      mapped_next[state_permute(state, bit_orders[permutation])] =
        state_permute(positive_next[state], bit_orders[permutation]);
    }
    mapped_state = state_permute(1U, bit_orders[permutation]);
    assert(motor_hall_decoder_init(mapped_next, 6000U,
                                   mapped_state, 0U));
    for (edge = 1U; edge <= 7U; edge++)
    {
      mapped_state = mapped_next[mapped_state];
      assert(motor_hall_decoder_edge_process(mapped_state, edge,
                                             edge * 100U));
    }
    sample = sample_get();
    assert(sample.direction == 1);
    assert(sample.electrical_frequency_millihz == 10000U);
  }
}

/**
 * @brief 验证反向序列、重复事件、非法电平、跨状态跳变和方向切换处理。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void transition_exception_test(void)
{
  motor_hall_sample_t sample;

  assert(motor_hall_decoder_init(positive_next, 6000U, 1U, 0U));
  assert(motor_hall_decoder_edge_process(3U, 1U, 100U));
  assert(motor_hall_decoder_edge_process(2U, 2U, 200U));
  sample = sample_get();
  assert(sample.direction == -1);
  assert(sample.negative_count == 2U);
  assert(sample.electrical_frequency_millihz == 10000U);

  assert(motor_hall_decoder_edge_process(3U, 3U, 300U));
  sample = sample_get();
  assert(sample.direction == 1);
  assert(sample.electrical_frequency_millihz == 0U);
  assert(motor_hall_decoder_edge_process(1U, 4U, 400U));
  sample = sample_get();
  assert(sample.electrical_frequency_millihz == 10000U);

  assert(!motor_hall_decoder_edge_process(1U, 5U, 410U));
  assert(!motor_hall_decoder_edge_process(0U, 6U, 420U));
  assert(!motor_hall_decoder_edge_process(5U, 7U, 430U));
  assert(!motor_hall_decoder_edge_process(2U, 8U, 440U));
  sample = sample_get();
  assert(sample.duplicate_count == 1U);
  assert(sample.invalid_transition_count == 3U);
  assert(sample.direction == 0);
  assert(sample.electrical_frequency_millihz == 0U);

  assert(motor_hall_decoder_edge_process(3U, 9U, 500U));
  sample = sample_get();
  assert(sample.direction == 1);
  assert(sample.electrical_frequency_millihz == 0U);
  assert(motor_hall_decoder_edge_process(1U, 10U, 600U));
  sample = sample_get();
  assert(sample.electrical_frequency_millihz == 10000U);
}

/**
 * @brief 验证无符号周期相减可正确跨越DWT 32位回绕点。
 * @param 无。
 * @return 无；失败时由assert终止测试。
 */
static void timestamp_wrap_test(void)
{
  motor_hall_sample_t sample;

  assert(motor_hall_decoder_init(positive_next, 9600U, 1U, 0xFFFFFF00U));
  assert(motor_hall_decoder_edge_process(5U, 1U, 0xFFFFFF80U));
  assert(motor_hall_decoder_edge_process(4U, 2U, 0x00000020U));
  sample = sample_get();
  assert(sample.electrical_frequency_millihz == 10000U);
}

int main(void)
{
  const uint8_t invalid_next[8] = {0U, 2U, 1U, 4U, 3U, 6U, 5U, 0U};
  const uint8_t non_adjacent_next[8] =
    {0U, 2U, 4U, 6U, 3U, 1U, 5U, 0U};

  assert(!motor_hall_decoder_init(invalid_next, 6000U, 1U, 0U));
  assert(!motor_hall_decoder_init(non_adjacent_next, 6000U, 1U, 0U));
  positive_rotation_test();
  hall_permutation_test();
  transition_exception_test();
  timestamp_wrap_test();
  printf("motor_hall_decoder_test: PASS\n");
  return 0;
}
