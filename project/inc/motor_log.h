#ifndef MOTOR_LOG_H
#define MOTOR_LOG_H

#include <stdint.h>
#include <stdio.h>

typedef enum
{
  MOTOR_LOG_LEVEL_OFF   = 0,
  MOTOR_LOG_LEVEL_ERROR = 1,
  MOTOR_LOG_LEVEL_WARN  = 2,
  MOTOR_LOG_LEVEL_DEBUG = 3,
  MOTOR_LOG_LEVEL_INFO  = 4
} motor_log_level_t;

extern volatile uint8_t motor_log_level;

void motor_log_level_set(motor_log_level_t level);
motor_log_level_t motor_log_level_get(void);

#define MOTOR_LOG_PRINT(required_level, tag, format, ...)                  \
  do                                                                       \
  {                                                                        \
    if (motor_log_level >= (uint8_t)(required_level))                       \
    {                                                                      \
      printf("[" tag "] %s %d: " format, __FUNCTION__, __LINE__,          \
             ##__VA_ARGS__);                                               \
    }                                                                      \
  } while (0)

#define LOGE(format, ...) MOTOR_LOG_PRINT(MOTOR_LOG_LEVEL_ERROR, "ERROR", format, ##__VA_ARGS__)
#define LOGW(format, ...) MOTOR_LOG_PRINT(MOTOR_LOG_LEVEL_WARN,  "WARN",  format, ##__VA_ARGS__)
#define LOGD(format, ...) MOTOR_LOG_PRINT(MOTOR_LOG_LEVEL_DEBUG, "DEBUG", format, ##__VA_ARGS__)
#define LOGI(format, ...) MOTOR_LOG_PRINT(MOTOR_LOG_LEVEL_INFO,  "INFO",  format, ##__VA_ARGS__)

#endif /* MOTOR_LOG_H */
