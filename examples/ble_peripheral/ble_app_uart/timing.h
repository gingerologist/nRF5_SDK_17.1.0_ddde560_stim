#ifndef __TIMING_H__
#define __TIMING_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_util.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"

extern nrf_drv_spi_t const m_dac_spi;
extern nrf_drv_spi_config_t const m_dac_spi_config;

extern nrf_drv_spi_config_t const m_dac_spi_config;
extern nrf_drv_spi_config_t const m_dac_spi_config_noss;

extern nrf_drv_timer_t const m_seg_counter;
extern nrf_drv_timer_t const m_cyc_counter;
extern nrf_drv_timer_t const m_spi_timer;
extern nrf_drv_timer_t const m_seg_timer;

void ss_pin_init(void);

void spi_timer_init(nrfx_timer_event_handler_t cb);

void count_timer_init(nrfx_timer_event_handler_t cb);
void count_timer_compare(uint32_t c0_val);

void spi_timer_compare2(uint32_t c0_val, uint32_t c1_val);
void spi_timer_compare3(uint32_t c0_val, uint32_t c1_val, uint32_t c2_val);
void spi_timer_compare4(uint32_t c0_val, uint32_t c1_val, uint32_t c2_val, uint32_t c3_val);
void spi_timer_compare5(uint32_t c0_val, uint32_t c1_val, uint32_t c2_val, uint32_t c3_val, uint32_t c4_val);

void spi_timer_c0_trigger_spi_task(void);
void spi_timer_c1_trigger_ss_and_count(void);
void spi_timer_c1_trigger_ss(void);
void spi_timer_c2_trigger_ss_and_count(void);
void spi_timer_c2_trigger_count(void);
void spi_timer_c3_trigger_ss(void);

void count_timer_c0_stop_spi_timer(void);
void count_timer_c0_stop_and_clear_spi_timer(void);

void spi_end_trigger_ss(void);
void spi_end_trigger_ss_and_clear_spi_timer(void);
void cycle_timer_c0_trigger_spi_timer(void);

#endif
