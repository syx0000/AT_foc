#include <stddef.h>
#include <string.h>
#include "flash_port.h"
#include "motor_control.h"
#include "motor_parameter.h"
#include "motor_parameter_storage.h"
#include "motor_pwm_port.h"

#define MOTOR_PARAMETER_STORAGE_MAGIC          0x4D50524DU
#define MOTOR_PARAMETER_STORAGE_FORMAT_VERSION 1U
#define MOTOR_PARAMETER_STORAGE_COMMIT_MARKER  0x434F4D54U
#define MOTOR_PARAMETER_STORAGE_SLOT_A_ADDR    FLASH_USER_START_ADDR
#define MOTOR_PARAMETER_STORAGE_SLOT_B_ADDR    \
  (FLASH_USER_START_ADDR + FLASH_SECTOR_SIZE)

typedef struct
{
  uint32_t magic;
  uint16_t format_version;
  uint16_t length;
  uint32_t sequence;
  motor_parameter_t parameter;
  uint32_t crc32;
  uint32_t commit_marker;
} motor_parameter_storage_record_t;

typedef char motor_parameter_record_must_fit_one_sector[
  (sizeof(motor_parameter_storage_record_t) <= FLASH_SECTOR_SIZE) ? 1 : -1];

static motor_parameter_storage_status_t motor_parameter_storage_status;

/** @brief 判断序号a是否比b更新。@param a 第一个序号。@param b 第二个序号。@return a更新返回true。 */
static bool motor_parameter_storage_sequence_newer(uint32_t a, uint32_t b)
{
  return ((int32_t)(a - b) > 0);
}

/** @brief 校验RAM记录的格式、提交标记、CRC和参数内容。@param record 待校验记录。@return 完整有效返回true。 */
static bool motor_parameter_storage_record_valid(
  const motor_parameter_storage_record_t *record)
{
  uint32_t crc;

  if ((record->magic != MOTOR_PARAMETER_STORAGE_MAGIC) ||
      (record->format_version != MOTOR_PARAMETER_STORAGE_FORMAT_VERSION) ||
      (record->length != sizeof(motor_parameter_t)) ||
      (record->commit_marker != MOTOR_PARAMETER_STORAGE_COMMIT_MARKER) ||
      (!motor_parameter_validate(&record->parameter))) return false;
  crc = flash_port_crc32(record, offsetof(motor_parameter_storage_record_t, crc32));
  return (crc == record->crc32);
}

/** @brief 读取并校验指定槽。@param address 槽首地址。@param record 输出记录。@return 记录有效返回true。 */
static bool motor_parameter_storage_slot_read(
  uint32_t address, motor_parameter_storage_record_t *record)
{
  flash_port_read(address, record, sizeof(*record));
  return motor_parameter_storage_record_valid(record);
}

/** @brief 扫描双槽并返回最新记录。@param record 输出最新有效记录。@return 存在有效槽返回true。 */
static bool motor_parameter_storage_scan(
  motor_parameter_storage_record_t *record)
{
  motor_parameter_storage_record_t slot_a;
  motor_parameter_storage_record_t slot_b;
  bool valid_a = motor_parameter_storage_slot_read(
    MOTOR_PARAMETER_STORAGE_SLOT_A_ADDR, &slot_a);
  bool valid_b = motor_parameter_storage_slot_read(
    MOTOR_PARAMETER_STORAGE_SLOT_B_ADDR, &slot_b);

  motor_parameter_storage_status.slot_a_valid = valid_a;
  motor_parameter_storage_status.slot_b_valid = valid_b;
  if (!valid_a && !valid_b)
  {
    motor_parameter_storage_status.source = MOTOR_PARAMETER_STORAGE_DEFAULTS;
    motor_parameter_storage_status.sequence = 0U;
    return false;
  }
  if (valid_b && (!valid_a || motor_parameter_storage_sequence_newer(
                    slot_b.sequence, slot_a.sequence)))
  {
    *record = slot_b;
    motor_parameter_storage_status.source = MOTOR_PARAMETER_STORAGE_SLOT_B;
  }
  else
  {
    *record = slot_a;
    motor_parameter_storage_status.source = MOTOR_PARAMETER_STORAGE_SLOT_A;
  }
  motor_parameter_storage_status.sequence = record->sequence;
  return true;
}

/** @brief 检查运行时Flash参数操作的电机安全条件。@param 无。@return READY且PWM关闭返回true。 */
static bool motor_parameter_storage_ready(void)
{
  motor_control_status_t control;

  return motor_control_status_read(&control) &&
    (control.state == MOTOR_CONTROL_STATE_READY) &&
    (!motor_pwm_port_output_is_enabled());
}

bool motor_parameter_storage_init(void)
{
  motor_parameter_storage_record_t record;

  if (!motor_parameter_storage_scan(&record)) return false;
  if (!motor_parameter_boot_load(&record.parameter))
  {
    motor_parameter_storage_status.source = MOTOR_PARAMETER_STORAGE_ERROR;
    return false;
  }
  return true;
}

bool motor_parameter_storage_load(void)
{
  motor_parameter_storage_record_t record;

  if (!motor_parameter_storage_ready() ||
      !motor_parameter_storage_scan(&record)) return false;
  return motor_parameter_boot_load(&record.parameter);
}

bool motor_parameter_storage_defaults(void)
{
  return motor_parameter_storage_ready() && motor_parameter_defaults_restore();
}

bool motor_parameter_storage_save(void)
{
  motor_parameter_storage_record_t current;
  motor_parameter_storage_record_t record;
  motor_parameter_storage_record_t verify;
  uint32_t target_address;
  uint32_t commit_address;
  uint32_t next_sequence;

  if (!motor_parameter_storage_ready()) return false;
  if (motor_parameter_storage_scan(&current))
  {
    next_sequence = current.sequence + 1U;
    target_address = (motor_parameter_storage_status.source ==
                      MOTOR_PARAMETER_STORAGE_SLOT_A) ?
                     MOTOR_PARAMETER_STORAGE_SLOT_B_ADDR :
                     MOTOR_PARAMETER_STORAGE_SLOT_A_ADDR;
  }
  else
  {
    next_sequence = 1U;
    target_address = MOTOR_PARAMETER_STORAGE_SLOT_A_ADDR;
  }

  memset(&record, 0xFF, sizeof(record));
  record.magic = MOTOR_PARAMETER_STORAGE_MAGIC;
  record.format_version = MOTOR_PARAMETER_STORAGE_FORMAT_VERSION;
  record.length = sizeof(motor_parameter_t);
  record.sequence = next_sequence;
  if (!motor_parameter_active_read(&record.parameter)) return false;
  record.crc32 = flash_port_crc32(
    &record, offsetof(motor_parameter_storage_record_t, crc32));

  if ((flash_port_sector_erase(target_address) != HAL_OK) ||
      (flash_port_write(target_address, &record,
        offsetof(motor_parameter_storage_record_t, commit_marker)) != HAL_OK))
    return false;
  flash_port_read(target_address, &verify, sizeof(verify));
  if ((verify.magic != record.magic) ||
      (verify.format_version != record.format_version) ||
      (verify.length != record.length) ||
      (verify.sequence != record.sequence) ||
      (verify.crc32 != record.crc32) ||
      (memcmp(&verify.parameter, &record.parameter,
              sizeof(record.parameter)) != 0) ||
      (flash_port_crc32(&verify,
        offsetof(motor_parameter_storage_record_t, crc32)) != verify.crc32))
    return false;

  commit_address = target_address +
    offsetof(motor_parameter_storage_record_t, commit_marker);
  record.commit_marker = MOTOR_PARAMETER_STORAGE_COMMIT_MARKER;
  if (flash_port_write(commit_address, &record.commit_marker,
                      sizeof(record.commit_marker)) != HAL_OK) return false;
  if (!motor_parameter_storage_slot_read(target_address, &verify)) return false;
  return motor_parameter_storage_scan(&current);
}

bool motor_parameter_storage_status_read(motor_parameter_storage_status_t *status)
{
  if (status == NULL) return false;
  *status = motor_parameter_storage_status;
  return true;
}
