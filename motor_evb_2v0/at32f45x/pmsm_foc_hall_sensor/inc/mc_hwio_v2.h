/**
  **************************************************************************
  * @file     mc_hwio_v2.h
  * @brief    Definition and declaration of Hardware peripheral configuration
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __MC_HWIO_V2_H
#define __MC_HWIO_V2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mc_lib.h"

/**************** define Timer for PWM ******************/
/* 3-phase complementary pwm pin definition */
#define PWM_ADVANCE_TIMER                   TMR1
#define PWM_ADVANCE_TIMER_CRM_CLK           CRM_TMR1_PERIPH_CLOCK
#define ADVTMR_PWM_CYCLE_IRQ                TMR1_OVF_TMR10_IRQHandler
#define ADVTMR_PWM_CYCLE_IRQn               TMR1_OVF_TMR10_IRQn
#define ADVTMR_PWM_BRK_IRQ                  TMR1_BRK_TMR9_IRQHandler
#define ADVTMR_PWM_BRK_IRQn                 TMR1_BRK_TMR9_IRQn

/* timer for adc trigger source */
#define ADC_TIMER                           TMR1
#define ADC_TIMER_SELECT_CHANNEL            TMR_SELECT_CHANNEL_4
#define TMR_ADC_TRIG_SOURCE                 ADC_PREEMPT_TRIG_TMR1CH4
#define TMR_ADC_TRIG_SIGNAL                 ADC_PREEMPT_TRIG_EDGE_RISING
#define TMR_ADC_TRIG_NO_SIGNAL              ADC_PREEMPT_TRIG_EDGE_NONE
/* define ADC TRIG OUTPUT PIN */
#define TMR_ADC_TRIG_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define TMR_ADC_TRIG_PORT                   GPIOA
#define TMR_ADC_TRIG_GPIO_PIN               GPIO_PINS_11
#define TMR_ADC_TRIG_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE11
#define TMR_ADC_TRIG_IOMUX                  GPIO_MUX_1

#ifdef ONE_SHUNT
/* tmr adc dma definition */
#define TMR_ADC_DMA_CRM_CLK                 CRM_DMA1_PERIPH_CLOCK
#define TMR_ADC_DMA_CH                      DMA1_CHANNEL4
#define TMR_ADC_DMA                         DMA1
#define TMR_ADC_DMA_FLEX                    DMAMUX_DMAREQ_ID_TMR1_CH4
#define TMR_ADC_DMA_FLEX_CH                 DMA1MUX_CHANNEL4
#define TMR_CH_ADC_DMA_REQUEST              TMR_C4_DMA_REQUEST
#define TMR_ADC_DMA_PERIPHERAL_ADDR         ADC_TIMER->c4dt
#endif

/**************** define GPIO for PWM *******************/
/***********************************************************
The definitions must be established in the following order:
    TMRx_CH1  -> PHASE_A_HI
    TMRx_CH1C -> PHASE_A_LOW
    TMRx_CH2  -> PHASE_B_HI
    TMRx_CH2C -> PHASE_B_LOW
    TMRx_CH3  -> PHASE_C_HI
    TMRx_CH3C -> PHASE_C_LOW
************************************************************/
#define PWM_PHASE_A_HI_GPIO_CRM_CLK         CRM_GPIOA_PERIPH_CLOCK
#define PWM_PHASE_A_HI_PORT                 GPIOA
#define PWM_PHASE_A_HI_GPIO_PIN             GPIO_PINS_8
#define PWM_PHASE_A_HI_PIN_SOURCE           GPIO_PINS_SOURCE8
#define PWM_PHASE_A_HI_IOMUX                GPIO_MUX_1
#define PWM_PHASE_B_HI_GPIO_CRM_CLK         CRM_GPIOA_PERIPH_CLOCK
#define PWM_PHASE_B_HI_PORT                 GPIOA
#define PWM_PHASE_B_HI_GPIO_PIN             GPIO_PINS_9
#define PWM_PHASE_B_HI_PIN_SOURCE           GPIO_PINS_SOURCE9
#define PWM_PHASE_B_HI_IOMUX                GPIO_MUX_1
#define PWM_PHASE_C_HI_GPIO_CRM_CLK         CRM_GPIOA_PERIPH_CLOCK
#define PWM_PHASE_C_HI_PORT                 GPIOA
#define PWM_PHASE_C_HI_GPIO_PIN             GPIO_PINS_10
#define PWM_PHASE_C_HI_PIN_SOURCE           GPIO_PINS_SOURCE10
#define PWM_PHASE_C_HI_IOMUX                GPIO_MUX_1
#define PWM_PHASE_A_LOW_GPIO_CRM_CLK        CRM_GPIOA_PERIPH_CLOCK
#define PWM_PHASE_A_LOW_PORT                GPIOA
#define PWM_PHASE_A_LOW_GPIO_PIN            GPIO_PINS_7
#define PWM_PHASE_A_LOW_PIN_SOURCE          GPIO_PINS_SOURCE7
#define PWM_PHASE_A_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_B_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_B_LOW_PORT                GPIOB
#define PWM_PHASE_B_LOW_GPIO_PIN            GPIO_PINS_0
#define PWM_PHASE_B_LOW_PIN_SOURCE          GPIO_PINS_SOURCE0
#define PWM_PHASE_B_LOW_IOMUX               GPIO_MUX_1
#define PWM_PHASE_C_LOW_GPIO_CRM_CLK        CRM_GPIOB_PERIPH_CLOCK
#define PWM_PHASE_C_LOW_PORT                GPIOB
#define PWM_PHASE_C_LOW_GPIO_PIN            GPIO_PINS_1
#define PWM_PHASE_C_LOW_PIN_SOURCE          GPIO_PINS_SOURCE1
#define PWM_PHASE_C_LOW_IOMUX               GPIO_MUX_1
#define PWM_BRK_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
#define PWM_BRK_PORT                        GPIOB
#define PWM_BRK_GPIO_PIN                    GPIO_PINS_12
#define PWM_BRK_GPIO_PIN_SOURCE             GPIO_PINS_SOURCE12
#define PWM_BRK_IOMUX                       GPIO_MUX_1


/**************** define Timer for Hall ******************/
/* hall sensor pin definition - target board: PB6/PB7/PB8 on TMR4 */
#define HALL_CAPTURE_TIMER                  TMR4
#define HALL_CAPTURE_CRM_CLK                CRM_TMR4_PERIPH_CLOCK
#define HALL_CAPTURE_IRQ                    TMR4_GLOBAL_IRQHandler
#define HALL_CAPTURE_IRQn                   TMR4_GLOBAL_IRQn
#define HALL_CAPTURE_FILTER_CLK_DIV         TMR_CLOCK_DIV2
#define TMR_HALL_IN_FILTER                  0x6                        /* 0x0 ~ 0xF */

/**************** define GPIO for Hall *******************/
#define HALL_A_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_A_PORT                         GPIOB
#define HALL_A_GPIO_PIN                     GPIO_PINS_6
#define HALL_A_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE6
#define HALL_A_IOMUX                        GPIO_MUX_2
#define HALL_B_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_B_PORT                         GPIOB
#define HALL_B_GPIO_PIN                     GPIO_PINS_7
#define HALL_B_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE7
#define HALL_B_IOMUX                        GPIO_MUX_2
#define HALL_C_GPIO_CRM_CLK                 CRM_GPIOB_PERIPH_CLOCK
#define HALL_C_PORT                         GPIOB
#define HALL_C_GPIO_PIN                     GPIO_PINS_8
#define HALL_C_GPIO_PIN_SOURCE              GPIO_PINS_SOURCE8
#define HALL_C_IOMUX                        GPIO_MUX_2

/**************** define GPIO for Encoder *******************/
/* encoder sensor pin definition */
#define ENCODER_MODE_TIMER                  TMR2
#define ENCODER_MODE_CRM_CLK                CRM_TMR2_PERIPH_CLOCK
#define ENCODER_A_GPIO_CRM_CLK              CRM_GPIOH_PERIPH_CLOCK
#define ENCODER_A_PORT                      GPIOH
#define ENCODER_A_GPIO_PIN                  GPIO_PINS_2
#define ENCODER_A_GPIO_PIN_SOURCE           GPIO_PINS_SOURCE2
#define ENCODER_A_IOMUX                     GPIO_MUX_1
#define ENCODER_B_GPIO_CRM_CLK              CRM_GPIOH_PERIPH_CLOCK
#define ENCODER_B_PORT                      GPIOH
#define ENCODER_B_GPIO_PIN                  GPIO_PINS_3
#define ENCODER_B_GPIO_PIN_SOURCE           GPIO_PINS_SOURCE3
#define ENCODER_B_IOMUX                     GPIO_MUX_1

#if defined ABZ || defined MAGNET_ENCODER_W_ABZ
/* encoder index */
#define ENCODER_Z_GPIO_CRM_CLK              CRM_GPIOD_PERIPH_CLOCK
#define ENCODER_Z_PORT                      GPIOD
#define ENCODER_Z_GPIO_PIN                  GPIO_PINS_2
#define ENCODER_Z_PORT_SOURCE               SCFG_PORT_SOURCE_GPIOD
#define ENCODER_Z_GPIO_PIN_SOURCE           SCFG_PINS_SOURCE2
#define ENCODER_Z_EXINT_CRM_CLK             CRM_SCFG_PERIPH_CLOCK
#define ENCODER_Z_EXINT_LINE                EXINT_LINE_2
#define EXINT_ENCODER_IDX_IRQ               EXINT2_IRQHandler
#define EXINT_ENCODER_IDX_IRQn              EXINT2_IRQn
#endif

/**************** define Timer and GPIO for ENCODER CAPTURE *******************/
#define ENCODER_CAPTURE_TIMER               TMR3
#define ENCODER_CAPTURE_CRM_CLK             CRM_TMR3_PERIPH_CLOCK
#define ENCODER_CAPTURE_IRQ                 TMR3_GLOBAL_IRQHandler
#define ENCODER_CAPTURE_IRQn                TMR3_GLOBAL_IRQn
#define ENCODER_CAPTURE_FILTER_CLK_DIV      TMR_CLOCK_DIV2
#define TMR_ENCODER_IN_FILTER               0x2                        /* 0x0 ~ 0xF */
#define ENC_A_GPIO_CRM_CLK                  CRM_GPIOB_PERIPH_CLOCK
#define ENC_A_PORT                          GPIOB
#define ENC_A_GPIO_PIN                      GPIO_PINS_4
#define ENC_A_GPIO_PIN_SOURCE               GPIO_PINS_SOURCE4
#define ENC_A_IOMUX                         GPIO_MUX_2
#define ENC_B_GPIO_CRM_CLK                  CRM_GPIOB_PERIPH_CLOCK
#define ENC_B_PORT                          GPIOB
#define ENC_B_GPIO_PIN                      GPIO_PINS_5
#define ENC_B_GPIO_PIN_SOURCE               GPIO_PINS_SOURCE5
#define ENC_B_IOMUX                         GPIO_MUX_2

/**************** define Synchronous Timer for read Magnetic encoder value *******************/
#define SYNC_TIMER                          TMR4
#define SYNC_TIMER_SELECT_CHANNEL           TMR_SELECT_CHANNEL_1
#define SYNC_TIMER_CRM_CLK                  CRM_TMR4_PERIPH_CLOCK
#define SYNC_TIMER_INT_SOURCE               TMR_SUB_INPUT_SEL_IS3
#define SYNC_TIMER_CH_INT                   TMR_C1_INT
#define SYNC_TIMER_CH_FLAG                  TMR_C1_FLAG
#define SYNC_TIMER_CH_IRQ                   TMR4_GLOBAL_IRQHandler
#define SYNC_TIMER_CH_IRQn                  TMR4_GLOBAL_IRQn

/**************** define Timer for speed control loop *******************/
#define SPEED_LOOP_TIMER                    TMR7
#define SPEED_LOOP_TIMER_CRM_CLK            CRM_TMR7_PERIPH_CLOCK
#define SPEED_LOOP_TIMER_IRQ                TMR7_GLOBAL_IRQHandler
#define SPEED_LOOP_TIMER_IRQn               TMR7_GLOBAL_IRQn

/* adc reading pin definition */
#define ADC_CONVERTER                       ADC1
#define ADC_CONVERTER_CRM_CLK               CRM_ADC1_PERIPH_CLOCK
#define ADC_CONVERTER_CRM_CLK_DIV           ADC_HCLK_DIV_4
#define ADC_SHUNT_SAMP_READY_IRQ            ADC1_2_IRQHandler
#define ADC_SHUNT_SAMP_READY_IRQn           ADC1_2_IRQn
#define ADC_ORDINARY_CH_LEN                 ADC_IDX_MAX
#define ADC_PREEMPT_SAMPLETIME              ADC_SAMPLETIME_6_5
#define ADC_SIMULTANE_CONVERTER             ADC2
#define ADC_SIMULTANE_CONVERTER_CRM_CLK     CRM_ADC2_PERIPH_CLOCK

/* dma1 ch1 for adc ordinary conversion */
#define ADC_ORDINARY_DMA_CRM_CLK            CRM_DMA1_PERIPH_CLOCK
#define ADC_ORDINARY_DMA_CHANNEL            DMA1_CHANNEL1
#define ADC_ORDINARY_DMA                    DMA1
#define ADC_ORDINARY_DMA_FLEX               DMAMUX_DMAREQ_ID_ADC1
#define ADC_ORDINARY_DMA_FLEX_CH            DMA1MUX_CHANNEL1
#define ADC_ORDINARY_DMA_FT_STS_FLAG        DMA1_FDT1_FLAG

#define CURR_PHASE_A_ADC_CH                 ADC_CHANNEL_0
#define CURR_PHASE_A_ADC_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK
#define CURR_PHASE_A_ADC_PORT               GPIOA
#define CURR_PHASE_A_ADC_GPIO_PIN           GPIO_PINS_0

#define CURR_PHASE_B_ADC_CH                 ADC_CHANNEL_1
#define CURR_PHASE_B_ADC_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK
#define CURR_PHASE_B_ADC_PORT               GPIOA
#define CURR_PHASE_B_ADC_GPIO_PIN           GPIO_PINS_1

#define CURR_PHASE_C_ADC_CH                 ADC_CHANNEL_2
#define CURR_PHASE_C_ADC_GPIO_CRM_CLK       CRM_GPIOA_PERIPH_CLOCK
#define CURR_PHASE_C_ADC_PORT               GPIOA
#define CURR_PHASE_C_ADC_GPIO_PIN           GPIO_PINS_2

#define CURR_BUS_ADC_CH                     ADC_CHANNEL_3
#define CURR_BUS_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define CURR_BUS_ADC_PORT                   GPIOA
#define CURR_BUS_ADC_GPIO_PIN               GPIO_PINS_3

#define VOLT_BUS_ADC_CH                     ADC_CHANNEL_3
#define VOLT_BUS_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define VOLT_BUS_ADC_PORT                   GPIOA
#define VOLT_BUS_ADC_GPIO_PIN               GPIO_PINS_3

#define MOS_TEMP_ADC_CH                     ADC_CHANNEL_5
#define MOS_TEMP_ADC_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define MOS_TEMP_ADC_PORT                   GPIOA
#define MOS_TEMP_ADC_GPIO_PIN               GPIO_PINS_5

#define POTENTIO_ADC_CH                     ADC_CHANNEL_10
#define POTENTIO_ADC_GPIO_CRM_CLK           CRM_GPIOC_PERIPH_CLOCK
#define POTENTIO_ADC_PORT                   GPIOC
#define POTENTIO_ADC_GPIO_PIN               GPIO_PINS_0

#define IBUS_AVG_ADC_CH                     ADC_CHANNEL_13
#define IBUS_AVG_ADC_GPIO_CRM_CLK           CRM_GPIOC_PERIPH_CLOCK
#define IBUS_AVG_ADC_PORT                   GPIOC
#define IBUS_AVG_ADC_GPIO_PIN               GPIO_PINS_3

#if defined VOLT_SENSE || defined WIND_SENSE
#define BEMF_A_ADC_CH                       ADC_CHANNEL_5
#define BEMF_A_ADC_GPIO_CRM_CLK             CRM_GPIOA_PERIPH_CLOCK
#define BEMF_A_ADC_PORT                     GPIOA
#define BEMF_A_ADC_GPIO_PIN                 GPIO_PINS_5

#define BEMF_B_ADC_CH                       ADC_CHANNEL_6
#define BEMF_B_ADC_GPIO_CRM_CLK             CRM_GPIOA_PERIPH_CLOCK
#define BEMF_B_ADC_PORT                     GPIOA
#define BEMF_B_ADC_GPIO_PIN                 GPIO_PINS_6

#define BEMF_C_ADC_CH                       ADC_CHANNEL_7
#define BEMF_C_ADC_GPIO_CRM_CLK             CRM_GPIOA_PERIPH_CLOCK
#define BEMF_C_ADC_PORT                     GPIOA
#define BEMF_C_ADC_GPIO_PIN                 GPIO_PINS_7
#endif

/**************** define comm uart Tx and Rx ******************/
#define COMM_UART                           USART1
#define COMM_UART_CRM_CLK                   CRM_USART1_PERIPH_CLOCK
#define COMM_UART_TX_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define COMM_UART_TX_PORT                   GPIOA
#define COMM_UART_TX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE15
#define COMM_UART_TX_PIN                    GPIO_PINS_15
#define COMM_UART_TX_IOMUX                  GPIO_MUX_7
#define COMM_UART_RX_GPIO_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK
#define COMM_UART_RX_PORT                   GPIOB
#define COMM_UART_RX_GPIO_PIN_SOURCE        GPIO_PINS_SOURCE3
#define COMM_UART_RX_PIN                    GPIO_PINS_3
#define COMM_UART_RX_IOMUX                  GPIO_MUX_7
#define COMM_UART_IOMUX                     (uint32_t)NULL
#define COMM_UART_IRQn                      USART1_IRQn
#define COMM_UART_IRQHandler                USART1_IRQHandler

/**************** define DMA for uart Tx and Rx ******************/
#define DMA_UART                            DMA1
#define DMA_UART_CRM_CLK                    CRM_DMA1_PERIPH_CLOCK
#define DMA_UART_TX_CHANNEL                 DMA1_CHANNEL2
#define DMA_UART_TX_FDT_FLAG                DMA1_FDT2_FLAG
#define DMA_UART_TX_FLEX_CHANNEL            DMA1MUX_CHANNEL2
#define DMA_UART_TX_FLEX                    DMAMUX_DMAREQ_ID_USART1_TX
#define DMA_UART_RX_CHANNEL                 DMA1_CHANNEL3
#define DMA_UART_RX_FDT_FLAG                DMA1_FDT3_FLAG
#define DMA_UART_RX_FLEX_CHANNEL            DMA1MUX_CHANNEL3
#define DMA_UART_RX_FLEX                    DMAMUX_DMAREQ_ID_USART1_RX

/******************* define led *******************/
/* error led state (target board: LED_ERR = PC14) */
#define  ERROR_LED_GPIO_CRM_CLK              CRM_GPIOC_PERIPH_CLOCK
#define  ERROR_LED_PORT                      GPIOC
#define  ERROR_LED_GPIO_PIN                  GPIO_PINS_14

/* adc trig led state */
#define  ADC_TRIG_LED_GPIO_CRM_CLK           CRM_GPIOA_PERIPH_CLOCK
#define  ADC_TRIG_LED_PORT                   GPIOA
#define  ADC_TRIG_LED_GPIO_PIN               GPIO_PINS_11

/* operating status LEDs (target board: LED_RUN = PC13) */
#define  STATUS1_LED_GPIO_CRM_CLK            CRM_GPIOC_PERIPH_CLOCK
#define  STATUS1_LED_PORT                    GPIOC
#define  STATUS1_LED_GPIO_PIN                GPIO_PINS_13
#define  STATUS2_LED_GPIO_CRM_CLK            CRM_GPIOC_PERIPH_CLOCK
#define  STATUS2_LED_PORT                    GPIOC
#define  STATUS2_LED_GPIO_PIN                GPIO_PINS_15
#define  STATUS3_LED_GPIO_CRM_CLK            CRM_GPIOB_PERIPH_CLOCK
#define  STATUS3_LED_PORT                    GPIOB
#define  STATUS3_LED_GPIO_PIN                GPIO_PINS_9

/******************* define EN_GATE for DRV8353 *******************/
#define  EN_GATE_GPIO_CRM_CLK                CRM_GPIOC_PERIPH_CLOCK
#define  EN_GATE_PORT                        GPIOC
#define  EN_GATE_GPIO_PIN                    GPIO_PINS_15

/******************* define button *******************/
typedef enum
{
  USER_BUTTON                               = 0,
  NO_BUTTON                                 = 1
} button_type;

#define USER_BUTTON_PIN                     GPIO_PINS_12
#define USER_BUTTON_PORT                    GPIOA
#define USER_BUTTON_CRM_CLK                 CRM_GPIOA_PERIPH_CLOCK

#define BUTTON_PORT_SOURCE                  SCFG_PORT_SOURCE_GPIOA
#define BUTTON_PIN_SOURCE                   SCFG_PINS_SOURCE12
#define BUTTON_EXINT_CRM_CLK                CRM_SCFG_PERIPH_CLOCK
#define BUTTON_EXINT_LINE                   EXINT_LINE_12
#define BUTTON_EXINT_IRQn                   EXINT15_10_IRQn
#define BUTTON_EXINT_IRQ                    EXINT15_10_IRQHandler

#define HALL_LEARN_BUTTON_PIN               GPIO_PINS_10
#define HALL_LEARN_BUTTON_PORT              GPIOB
#define HALL_LEARN_BUTTON_CRM_CLK           CRM_GPIOB_PERIPH_CLOCK

#define HALL_LEARN_BUTTON_PORT_SOURCE       SCFG_PORT_SOURCE_GPIOB
#define HALL_LEARN_BUTTON_PIN_SOURCE        SCFG_PINS_SOURCE10
#define HALL_LEARN_BUTTON_EXINT_CRM_CLK     CRM_SCFG_PERIPH_CLOCK
#define HALL_LEARN_BUTTON_EXINT_LINE        EXINT_LINE_10

#define MODE1_BUTTON_PIN                    GPIO_PINS_8
#define MODE1_BUTTON_PORT                   GPIOC
#define MODE1_BUTTON_CRM_CLK                CRM_GPIOC_PERIPH_CLOCK

#define MODE2_BUTTON_PIN                    GPIO_PINS_9
#define MODE2_BUTTON_PORT                   GPIOC
#define MODE2_BUTTON_CRM_CLK                CRM_GPIOC_PERIPH_CLOCK

#define REVERSE_SW_PIN                      MODE1_BUTTON_PIN
#define REVERSE_SW_PORT                     MODE1_BUTTON_PORT

#define BRAKE_SW_PIN                        MODE2_BUTTON_PIN
#define BRAKE_SW_PORT                       MODE2_BUTTON_PORT

#define LOCK_MOTOR_SW_PIN                   GPIO_PINS_11
#define LOCK_MOTOR_SW_PORT                  GPIOB
#define LOCK_MOTOR_SW_CRM_CLK               CRM_GPIOB_PERIPH_CLOCK

#define PARKING_LOCK_SW_PIN                 GPIO_PINS_3
#define PARKING_LOCK_SW_PORT                GPIOB
#define PARKING_LOCK_SW_CRM_CLK             CRM_GPIOB_PERIPH_CLOCK

/* pwm brake function */
#define BRAKE_PWM_TIMER                     TMR10
#define BRAKE_PWM_TIMER_CRM_CLK             CRM_TMR10_PERIPH_CLOCK
#define BRAKE_PWM_TIMER_CH                  TMR_SELECT_CHANNEL_1
#define BRAKE_PWM_GPIO_CRM_CLK              CRM_GPIOB_PERIPH_CLOCK
#define BRAKE_PWM_PORT                      GPIOB
#define BRAKE_PWM_PIN                       GPIO_PINS_8
#define BRAKE_PWM_PIN_SOURCE                GPIO_PINS_SOURCE8
#define BRAKE_PWM_IOMUX                     GPIO_MUX_3

typedef enum
{
#if defined VOLT_SENSE || defined WIND_SENSE
  ADC_BEMF_A_IDX,
  ADC_BEMF_B_IDX,
  ADC_BEMF_C_IDX,
#endif
  ADC_BUS_VOLT_IDX,
  ADC_MOS_TEMP_IDX,
  ADC_POTENTIO_IDX,
  ADC_IBUS_AVE_IDX,
  ADC_IDX_MAX
} adc_in_idx;

extern __IO uint16_t adc_in_tab[ADC_IDX_MAX];

void nvic_config(void);
void tmr_pwm_init(void);
void adc_ordinary_config(void);
void adc_preempt_config(void);
void encoder_timer_init(void);
void encoder_capture_timer_init(void);
void magnetic_encoder_timer_init(void);
void hall_timer_init(void);
void speed_timer_init(void);
void brake_pwm_init(void);
void uart_init(usart_config_type *usart_config);

/******************** functions ********************/
/* led operation function */
void led_init(void);
void led_on(gpio_type *led_gpio_port, uint16_t led_gpio_pin);
void led_off(gpio_type *led_gpio_port, uint16_t led_gpio_pin);
void led_toggle(gpio_type *led_gpio_port, uint16_t led_gpio_pin);
void led_config(void);
void led_blink(void);

/* mode switch configuration */
void mode_switch_init(void);
void gpio_pins_init(void);
/* button operation function */
void button_exint_init(void);

/* initial angle detection configuration */
void foc_angle_init_config(void);

/* motor parameter identify configuration */
void motor_parameter_ID_config(void);

void get_int_vref_cal_ratio(void);
#ifdef __cplusplus
}
#endif

#endif
