/**
 * @file    motor_parameter_storage_test.c
 * @brief   电机参数Flash双槽存储 单元测试
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "flash_port.h"
#include "motor_control.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"

static uint8_t test_flash[2U * FLASH_SECTOR_SIZE];
static bool test_running;
static bool test_pwm_enabled;
#include "../project/foc/foc_app/motor_parameter_storage.c"

HAL_StatusTypeDef flash_port_sector_erase(uint32_t addr)
{
  if ((addr < FLASH_USER_START_ADDR) ||
      (addr >= FLASH_USER_END_ADDR)) return HAL_ERROR;
  memset(&test_flash[addr - FLASH_USER_START_ADDR], 0xFF,
         FLASH_SECTOR_SIZE);
  return HAL_OK;
}

HAL_StatusTypeDef flash_port_write(uint32_t addr, const void *data, uint32_t len)
{
  uint32_t offset = addr - FLASH_USER_START_ADDR;
  uint32_t index;

  if ((addr < FLASH_USER_START_ADDR) ||
      (len > sizeof(test_flash) - offset)) return HAL_ERROR;
  for (index = 0U; index < len; index++) test_flash[offset + index] &=
    ((const uint8_t *)data)[index];
  return HAL_OK;
}

void flash_port_read(uint32_t addr, void *data, uint32_t len)
{
  memcpy(data, &test_flash[addr - FLASH_USER_START_ADDR], len);
}

uint32_t flash_port_crc32(const void *data, uint32_t len)
{
  const uint8_t *byte = data;
  uint32_t crc = 0xFFFFFFFFU;
  uint32_t bit;

  while (len-- != 0U)
  {
    crc ^= *byte++;
    for (bit = 0U; bit < 8U; bit++)
      crc = (crc >> 1) ^ (0xEDB88320U &
            (uint32_t)-(int32_t)(crc & 1U));
  }
  return ~crc;
}

bool motor_control_status_read(motor_control_status_t *status)
{
  memset(status, 0, sizeof(*status));
  status->state = test_running ? MOTOR_CONTROL_STATE_SPEED_CONTROL : MOTOR_CONTROL_STATE_READY;
  return true;
}

bool motor_pwm_port_output_is_enabled(void) { return test_pwm_enabled; }

#include "../project/foc/foc_app/motor_parameter.c"

/* Exercise the real parameter manager, validation, 64-bit diff and Flash migrations. */
int main(void)
{
  motor_parameter_t p, loaded;
  motor_parameter_diff_t diff;
  motor_parameter_storage_status_t status;
  motor_parameter_init();
  assert(motor_parameter_active_read(&p));
  assert(motor_parameter_validate(&p));
  assert(p.offset_iq_ma == 10000U && p.flux_iq_ma == 10000U);
  test_running=true;
  assert(!motor_parameter_candidate_24v_profile());
  test_running=false; test_pwm_enabled=true;
  assert(!motor_parameter_candidate_24v_profile());
  test_pwm_enabled=false;
  assert(motor_parameter_candidate_24v_profile());
  assert(motor_parameter_candidate_read(&loaded));
  assert(loaded.phase_resistance_mohm==80 && loaded.direct_inductance_uh==50);
  assert(loaded.quadrature_inductance_uh==50);
  assert(loaded.current_loop_bandwidth_hz==888);
  assert(loaded.current_d_kp_q15==9141 && loaded.current_q_kp_q15==9141);
  assert(loaded.current_ki_q15==914 && loaded.speed_current_limit_ma==35000);
  assert(loaded.speed_maximum_rpm==3000 && loaded.speed_acceleration_rpm_per_s==30000);
  assert(loaded.speed_deceleration_rpm_per_s==30000);
  assert(loaded.speed_startup_iq_slew_ma_s==200000);
  assert(loaded.flux_linkage_uwb==p.flux_linkage_uwb && loaded.speed_kp_q20==p.speed_kp_q20 && loaded.speed_ki_q20==p.speed_ki_q20);
  assert(memcmp(loaded.hall_entry_angle_u16,p.hall_entry_angle_u16,sizeof(p.hall_entry_angle_u16))==0);
  assert(motor_parameter_active_read(&loaded));
  assert(memcmp(&loaded,&p,sizeof(p))==0); /* never changes the running profile */
  motor_parameter_candidate_discard();
  assert(!motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_CAN_NODE_ID, 128));
  assert(!motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_OFFSET_VOLTAGE_PCT, 58));
  assert(!motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_STARTUP_TIMEOUT_MS, 100));
  assert(!motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_STARTUP_ALIGN_MA, 25000));
  assert(!motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_STARTUP_RAMP_MHZ_S, 20500));
  assert(motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_CAN_NODE_ID, 7));
  assert(motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_OFFSET_IQ_MA, 12000));
  assert(motor_parameter_debug_get(MOTOR_PARAMETER_FIELD_CAN_NODE_ID) == 1U);
  assert(motor_parameter_diff_read(&diff));
  assert((diff.scalar_fields & (1ULL << MOTOR_PARAMETER_FIELD_CAN_NODE_ID)) != 0U);
  assert(MOTOR_PARAMETER_FIELD_CAN_NODE_ID >= 32);
  test_running = true;
  assert(!motor_parameter_candidate_accept());
  assert(!motor_parameter_storage_save());
  test_running = false;
  test_pwm_enabled = true;
  assert(!motor_parameter_candidate_accept());
  assert(!motor_parameter_storage_save());
  test_pwm_enabled = false;
  assert(motor_parameter_candidate_accept());
  assert(motor_parameter_debug_get(MOTOR_PARAMETER_FIELD_CAN_NODE_ID) == 7U);
  assert(motor_parameter_active_read(&p));
  assert(p.offset_iq_ma == 12000U && p.flux_iq_ma == 10000U);
  memset(test_flash, 0xFF, sizeof(test_flash));
  assert(motor_parameter_storage_save());
  assert(motor_parameter_storage_status_read(&status));
  assert(status.sequence == 1U);
  motor_parameter_init();
  assert(motor_parameter_storage_init());
  assert(motor_parameter_active_read(&loaded));
  assert(memcmp(&p, &loaded, sizeof(p)) == 0);
  /* A second write uses the other slot. Damage it: valid first slot survives. */
  assert(motor_parameter_candidate_field_set(MOTOR_PARAMETER_FIELD_CAN_NODE_ID, 8));
  assert(motor_parameter_candidate_accept());
  assert(motor_parameter_storage_save());
  test_flash[FLASH_SECTOR_SIZE + offsetof(motor_parameter_storage_record_t, parameter)] ^= 1U;
  assert(motor_parameter_storage_init());
  assert(motor_parameter_debug_get(MOTOR_PARAMETER_FIELD_CAN_NODE_ID) == 7U);
  for (uint16_t version=2U; version<=6U; version++)
  {
    uint16_t length = (uint16_t)(version==2U ? MOTOR_PARAMETER_STORAGE_V2_LENGTH :
                      version==3U ? MOTOR_PARAMETER_STORAGE_V3_LENGTH :
                      version==4U ? MOTOR_PARAMETER_STORAGE_V4_LENGTH :
                      version==5U ? offsetof(motor_parameter_t, speed_deceleration_rpm_per_s) :
                      offsetof(motor_parameter_t, speed_startup_iq_slew_ma_s));
    motor_parameter_storage_record_t old;
    uint32_t crc, marker=MOTOR_PARAMETER_STORAGE_COMMIT_MARKER;
    uint32_t header=offsetof(motor_parameter_storage_record_t, parameter);
    memset(&old, 0, sizeof(old));
    old.magic=MOTOR_PARAMETER_STORAGE_MAGIC;
    old.format_version=version; old.length=length; old.sequence=42U;
    old.parameter=p;
    old.parameter.speed_kp_q20=4800;
    old.parameter.hall_rotor_offset_u16=1234U;
    crc=flash_port_crc32(&old, header+length);
    memset(test_flash, 0xFF, sizeof(test_flash));
    memcpy(test_flash, &old, header+length);
    memcpy(test_flash+header+length, &crc, 4);
    memcpy(test_flash+header+length+4, &marker, 4);
    motor_parameter_init();
    assert(motor_parameter_storage_init());
    assert(motor_parameter_active_read(&loaded));
    assert(loaded.hall_rotor_offset_u16 == 1234U && loaded.speed_kp_q20 == 4800);
    if (version < 5U) assert(loaded.startup_align_ma == MOTOR_CURRENT_STARTUP_ALIGNMENT_CURRENT_MA);
    assert(loaded.speed_deceleration_rpm_per_s == 30000U);
    assert(loaded.speed_startup_iq_slew_ma_s == 200000U);
    assert(loaded.offset_iq_ma == (version < 5U ? MOTOR_COMMISSIONING_IQ_REFERENCE_MA : p.offset_iq_ma));
    assert(loaded.can_node_id == (version < 5U ? MOTOR_CAN_NODE_ID : p.can_node_id));
    assert(motor_parameter_storage_save());
    assert(motor_parameter_storage_status_read(&status));
    assert(status.sequence == 43U);
    motor_parameter_init();
    assert(motor_parameter_storage_init());
    assert(motor_parameter_active_read(&p));
    assert(memcmp(&p, &loaded, sizeof(p)) == 0);
  }
  return 0;
}
