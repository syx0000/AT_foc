/* Host-only HAL/state stand-ins. No firmware build includes this directory. */
#include <stdint.h>
#define TRUE 1
#define FALSE 0
#define SET 1
#define RESET 0
#define SUCCESS 1
#define ERROR 0
#define MAX_SPEED_RPM 4000
#define MAX_CURRENT 50.0f
#define CURRENT_BASE 66.0f
#define MC_NO_ERROR 0
#define ESC_STATE_SAFETY_READY 1
#define ESC_STATE_RUNNING 2
#define SPEED_CTRL 3
#define CAN1 1
#define GPIOA 0
#define CRM_GPIOA_PERIPH_CLOCK 0
#define CRM_CAN1_PERIPH_CLOCK 0
#define GPIO_MODE_MUX 0
#define GPIO_OUTPUT_PUSH_PULL 0
#define GPIO_PULL_NONE 0
#define GPIO_DRIVE_STRENGTH_MODERATE 0
#define GPIO_PINS_11 0
#define GPIO_PINS_12 0
#define GPIO_PINS_SOURCE11 0
#define GPIO_PINS_SOURCE12 0
#define GPIO_MUX_9 0
#define CRM_CAN1 0
#define CRM_CAN_CLOCK_SOURCE_HEXT 0
#define CAN_RE_TRANS_TIMES_2 0
#define CAN_RE_ARBI_TIMES_2 0
#define CAN_MODE_COMMUNICATE 0
#define CAN_RXBUF_OVERFLOW_BE_LOSE 0
#define CAN_ID_STANDARD 0
#define CAN_FRAME_DATA 0
#define SUPPORT_CAN_FD
#define CAN_FORMAT_CLASSIC 0
#define CAN_DLC_BYTES_0 0
#define CAN_DLC_BYTES_1 1
#define CAN_DLC_BYTES_3 3
#define CAN_DLC_BYTES_4 4
#define CAN_DLC_BYTES_5 5
#define CAN_DLC_BYTES_6 6
#define CAN_DLC_BYTES_8 8
#define CAN_TXBUF_PTB 0
#define CAN_TRANSMIT_PTB 0
#define CAN_RXBUF_STATUS_EMPTY 0
typedef struct { int gpio_mode, gpio_out_type, gpio_pull, gpio_drive_strength, gpio_pins; } gpio_init_type;
typedef struct { int bittime_div, ac_bts1_size, ac_bts2_size, ac_rsaw_size; } can_bittime_type;
typedef struct { uint32_t id; int id_type, frame_type, data_length, fd_format; uint8_t data[64]; } can_rxbuf_type;
typedef can_rxbuf_type can_txbuf_type;
static struct { int32_t cmd_final, command; } speed_ramp;
static struct { int16_t max_current, min_current; struct { int16_t q; } Iqdref; } current;
static struct { int16_t upper_limit_output, lower_limit_output; int32_t integral, upper_limit_integral, lower_limit_integral; uint16_t ki_shift; } pid_spd;
static struct { int16_t q; } Ival;
static int start_stop_btn_flag, error_code, esc_state, ctrl_mode, ctrl_mode_cmd;
static int32_t rotor_speed_val;
#define crm_periph_clock_enable(...) ((void)0)
#define gpio_default_para_init(...) ((void)0)
#define gpio_init(...) ((void)0)
#define gpio_pin_mux_config(...) ((void)0)
#define crm_can_clock_select(...) ((void)0)
#define can_software_reset(...) ((void)0)
#define can_bittime_default_para_init(...) ((void)0)
#define can_bittime_set(...) ((void)0)
#define can_retransmission_limit_set(...) ((void)0)
#define can_rearbitration_limit_set(...) ((void)0)
#define can_mode_set(...) ((void)0)
#define can_receive_all_enable(...) ((void)0)
#define can_rxbuf_overflow_mode_set(...) ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(...) ((void)0)
static can_rxbuf_type rx_queue[16];
static unsigned rx_head, rx_tail;
static can_txbuf_type tx_queue[16];
static unsigned tx_count;
static int tx_busy;
static int can_rxbuf_status_get(int bus) { (void)bus; return rx_head != rx_tail; }
static int can_rxbuf_read(int bus, can_rxbuf_type *rx) { (void)bus; *rx = rx_queue[rx_head++]; return SUCCESS; }
static int can_txbuf_write(int bus, int buffer, can_txbuf_type *tx) {
  (void)bus; (void)buffer; if (tx_busy) return ERROR;
  tx_queue[tx_count++] = *tx; return SUCCESS;
}
static int can_txbuf_transmit(int bus, int buffer) { (void)bus; (void)buffer; return SUCCESS; }
