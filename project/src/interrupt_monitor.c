/**
 * @file    interrupt_monitor.c
 * @brief   中断频率与控制诊断统计 功能实现
 * @author  随永祥
 * @date    2026-09-04
 * @version 1.0
 */

#include "at32f45x.h"
#include "interrupt_monitor.h"

interrupt_monitor_counters_t interrupt_monitor_counters;
motor_performance_counter_t adc_fast_performance_counter;

