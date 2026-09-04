#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "flash_port.h"
#include "motor_control.h"
#include "motor_parameter.h"
#include "motor_pwm_port.h"

static uint8_t test_flash[2U * FLASH_SECTOR_SIZE];
static motor_parameter_t test_active_parameter;
static motor_parameter_t test_loaded_parameter;

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
  status->state = MOTOR_CONTROL_STATE_READY;
  return true;
}

bool motor_pwm_port_output_is_enabled(void) { return false; }
bool motor_parameter_active_read(motor_parameter_t *parameter)
{
  *parameter = test_active_parameter;
  return true;
}
bool motor_parameter_boot_load(const motor_parameter_t *parameter)
{
  test_loaded_parameter = *parameter;
  return true;
}
bool motor_parameter_defaults_restore(void) { return true; }
bool motor_parameter_validate(const motor_parameter_t *parameter)
{
  return (parameter != NULL) && (parameter->pole_pairs > 0U);
}

int main(void)
{
  motor_parameter_storage_status_t status;
  uint32_t commit_marker = MOTOR_PARAMETER_STORAGE_COMMIT_MARKER;
  uint32_t slot_b_commit = FLASH_SECTOR_SIZE +
    offsetof(motor_parameter_storage_record_t, commit_marker);

  memset(test_flash, 0xFF, sizeof(test_flash));
  memset(&test_active_parameter, 0, sizeof(test_active_parameter));
  test_active_parameter.pole_pairs = 4U;
  assert(!motor_parameter_storage_init());
  assert(motor_parameter_storage_save());
  assert(motor_parameter_storage_status_read(&status));
  assert(status.source == MOTOR_PARAMETER_STORAGE_SLOT_A);
  assert(status.sequence == 1U);

  test_active_parameter.pole_pairs = 7U;
  assert(motor_parameter_storage_save());
  assert(motor_parameter_storage_status_read(&status));
  assert(status.source == MOTOR_PARAMETER_STORAGE_SLOT_B);
  assert(status.sequence == 2U);
  assert(motor_parameter_storage_init());
  assert(test_loaded_parameter.pole_pairs == 7U);

  memset(&test_flash[slot_b_commit], 0xFF, sizeof(uint32_t));
  assert(motor_parameter_storage_init());
  assert(test_loaded_parameter.pole_pairs == 4U);
  memcpy(&test_flash[slot_b_commit], &commit_marker, sizeof(commit_marker));
  test_flash[FLASH_SECTOR_SIZE +
    offsetof(motor_parameter_storage_record_t, parameter)] ^= 1U;
  assert(motor_parameter_storage_init());
  assert(test_loaded_parameter.pole_pairs == 4U);
  return 0;
}
