/**
  **************************************************************************
  * @file     AS5047P.c
  * @brief    Magnetic encoder(AS5047P) configuration with spi communication
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

/* ----------------------------------------------------------------------
** Include Files
** ------------------------------------------------------------------- */
#include "mc_lib.h"

/** @addtogroup Motor_Control_Library
  * @{
  */

/** @defgroup AS5047P
  * @brief Magnetic encoder(AS5047P) configuration with spi communication
  * @{
  */

/**
  * @brief  Initialization of SPI
  * @param  none
  * @retval none
  */
void mag_encoder_spi_init(void)
{
  gpio_init_type gpio_init_struct;
  // Config CS pin
  crm_periph_clock_enable(MAG_ENCODER_SPI_CS_CRM_CLK, TRUE);
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_pin_mux_config(MAG_ENCODER_SPI_CS_GPIO_PORT, MAG_ENCODER_SPI_CS_GPIO_PIN_SOURCE, MAG_ENCODER_SPI_CS_IOMUX);
  gpio_pin_mux_config(MAG_ENCODER_SPI_CLK_GPIO_PORT, MAG_ENCODER_SPI_CLK_GPIO_PIN_SOURCE, MAG_ENCODER_SPI_CLK_IOMUX);
  gpio_pin_mux_config(MAG_ENCODER_SPI_MISO_GPIO_PORT, MAG_ENCODER_SPI_MISO_GPIO_PIN_SOURCE, MAG_ENCODER_SPI_MISO_IOMUX);
  gpio_pin_mux_config(MAG_ENCODER_SPI_MOSI_GPIO_PORT, MAG_ENCODER_SPI_MOSI_GPIO_PIN_SOURCE, MAG_ENCODER_SPI_MOSI_IOMUX);
#else
  gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);  // Release PB3(SWO) pin to gpio
#endif
  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pins           = MAG_ENCODER_SPI_CS_GPIO_PIN;
  gpio_init(MAG_ENCODER_SPI_CS_GPIO_PORT, &gpio_init_struct);
  // Config CLK pin
  crm_periph_clock_enable(MAG_ENCODER_SPI_CLK_CRM_CLK, TRUE);
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
  gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pins           = MAG_ENCODER_SPI_CLK_GPIO_PIN;
  gpio_init(MAG_ENCODER_SPI_CLK_GPIO_PORT, &gpio_init_struct);
  // Config MISO pin
  crm_periph_clock_enable(MAG_ENCODER_SPI_MISO_CRM_CLK, TRUE);
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
#if !defined AT32F403Axx && !defined AT32F407xx && !defined AT32F413xx && !defined AT32F415xx
  gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
#else
  gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
#endif
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pins           = MAG_ENCODER_SPI_MISO_GPIO_PIN;
  gpio_init(MAG_ENCODER_SPI_MISO_GPIO_PORT, &gpio_init_struct);
// Config MOSI pin
  crm_periph_clock_enable(MAG_ENCODER_SPI_MOSI_CRM_CLK, TRUE);
  gpio_init_struct.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
  gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_pins           = MAG_ENCODER_SPI_MOSI_GPIO_PIN;
  gpio_init(MAG_ENCODER_SPI_MOSI_GPIO_PORT, &gpio_init_struct);

  spi_init_type spi_init_struct;
  crm_periph_clock_enable(MAG_ENCODER_SPI_CRM_CLK, TRUE);
  spi_default_para_init(&spi_init_struct);
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  /* AS5047P spi max. clock = 10MHz */
#ifdef AT32F415xx
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;         /* spi clock for AT32F415xx = 75/8=9.375MHz */
#elif defined AT32M412xx || defined AT32M416xx || defined AT32F422xx || defined AT32F426xx || defined AT32F455xx || defined AT32F456xx || defined AT32F457xx || defined AT32F402xx || defined AT32F405xx
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;         /* spi clock for AT32M412xx = 180/32=5.625MHz
                                                                   spi clock for AT32F422xx = 180/32=5.625MHz
                                                                   spi clock for AT32F455xx = 192/32=6MHz
                                                                   spi clock for AT32F402xx = 216/32=6.75MHz       */
#else
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_16;        /* spi clock for AT32F413xx =  100/16=6.25MHz,
                                                                                AT32F403Axx = 120/16=7.5MHz,
                                                                                AT32F421xx =  120/16=7.5MHz,
                                                                                AT32F423xx =  150/16=9.375MHz,
                                                                                AT32F435xx =  144/16=9MHz,
                                                                                AT32F425xx =   96/16=6MHz         */
#endif
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(MAG_ENCODER_SPI_INTERFACE, &spi_init_struct);

  spi_enable(MAG_ENCODER_SPI_INTERFACE, TRUE);
}

/**
  * @brief  generate an even parity bit for a 15-bit data
  * @param  a 15-bit source data
  * @retval a even parity value
  */
uint16_t get_even_parity_data(uint16_t data)
{
  uint16_t frame, parity = 0;
  const uint16_t parity_change = (1u << 15);

  frame = data & 0x7FFF;

  while (frame)
  {
    parity ^= parity_change;
    frame = frame & (frame - 1);
  }

  frame = data | parity;

  return frame;
}

/**
  * @brief  SPI write and read data byte function
  * @param  TxData: transmit data byte
  * @retval receive data byte
  */
uint16_t SPI_ReadWriteByte(uint16_t TxData)
{
  uint8_t retry = 0;

  while(spi_i2s_flag_get(MAG_ENCODER_SPI_INTERFACE, SPI_I2S_TDBE_FLAG) == RESET)
  {
    retry++;

    if (retry > 200)
    {
      return 0;
    }
  }

  spi_i2s_data_transmit(MAG_ENCODER_SPI_INTERFACE, TxData);
  retry = 0;

  while(spi_i2s_flag_get(MAG_ENCODER_SPI_INTERFACE, SPI_I2S_RDBF_FLAG) == RESET)
  {
    retry++;

    if (retry > 200)
    {
      return 0;
    }
  }

  return spi_i2s_data_receive(MAG_ENCODER_SPI_INTERFACE);
}

/**
  * @brief  read the register value.
  * @param  RegValue: register address
  * @retval register data
  */
uint16_t ReadValue(uint16_t RegValue)
{
  const uint16_t nop_command = 0xC000;
  uint16_t frame;
  uint16_t data;

  MAG_ENCODER_SPI_CS_ENABLE;

  /* add a read-bit flag at 14th bit */
  frame = RegValue | (1u << 14);
  /* convert frame to an even parity data */
  frame = get_even_parity_data(frame);

  while(spi_i2s_flag_get(MAG_ENCODER_SPI_INTERFACE, SPI_I2S_TDBE_FLAG) == RESET);

  spi_i2s_data_transmit(MAG_ENCODER_SPI_INTERFACE, frame);

  while(spi_i2s_flag_get(MAG_ENCODER_SPI_INTERFACE, SPI_I2S_RDBF_FLAG) == RESET);

  MAG_ENCODER_SPI_CS_DISABLE;

  spi_i2s_data_receive(MAG_ENCODER_SPI_INTERFACE);

  mc_delay_us(1);

  MAG_ENCODER_SPI_CS_ENABLE;

  spi_i2s_data_transmit(MAG_ENCODER_SPI_INTERFACE, nop_command);

  while(spi_i2s_flag_get(MAG_ENCODER_SPI_INTERFACE, SPI_I2S_RDBF_FLAG) == RESET);

  MAG_ENCODER_SPI_CS_DISABLE;

  data = spi_i2s_data_receive(MAG_ENCODER_SPI_INTERFACE);

  return(data);
}

/**
  * @brief  read the angle value.
  * @param  none
  * @retval angle value
  */
uint16_t ReadAngle(void)
{
  return(ReadValue(READ_ANGLE_VALUE) & 0x3FFF);
}
