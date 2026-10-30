#ifndef MC_HOST_H
#define MC_HOST_H
#include <stdint.h>
#if defined(USE_HALL_HOST)
int mc_host_stop_requested(void);
void mc_host_rx_byte(uint8_t byte);
void mc_host_rx_error(void);
void mc_host_task(void);
void mc_host_sample(void);
void mc_host_tick(void);
#endif
#endif
