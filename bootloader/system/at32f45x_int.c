/**
  * @file    at32f45x_int.c
  * @brief   Bootloader interrupt handlers (minimal)
  */
#include "at32f45x.h"

void NMI_Handler(void) {}
void HardFault_Handler(void) { while(1); }
void MemManage_Handler(void) { while(1); }
void BusFault_Handler(void) { while(1); }
void UsageFault_Handler(void) { while(1); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) {}

/* USART1 IRQ - not used in blocking mode */
void USART1_IRQHandler(void)
{
}
