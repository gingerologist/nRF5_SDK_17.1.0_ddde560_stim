#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_ppi.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "../howland.h"
#include "../timing.h"
#include "test.h"

const uint32_t intervals[16] = {
    100,  200,  300,  400,  500,  600,  700,  800,
    900, 1000, 1100, 1200, 1300, 1400, 1500, 1600
};

static void test2a_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    static uint8_t index = 0;
    nrf_drv_timer_extended_compare(&m_seg_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   intervals[(index++) & 0x0f],
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
}

/*
 * This test uses timer to trigger ppi channel and to trigger spi cs pin in turn.
 * cs pin task is configured to be toggle
 * in callback, timer is dynamically reset to new timeout (compare) value
 * a square wave with increasing cycle will be spotted on cs pin.
 */
void test2a(void)
{
    uint32_t err, event_addr, task_addr;
    
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);

    // cycle timer init
    nrf_drv_timer_config_t cycle_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    
    err = nrf_drv_timer_init(&m_seg_timer, &cycle_cfg, test2a_cycle_timer_callback);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_extended_compare(&m_seg_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   1000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
                                   
    nrf_ppi_channel_t ppic;
    
    err = nrfx_ppi_channel_alloc(&ppic);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_seg_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppic, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppic);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_seg_timer);
}

static void test2b_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);
    // nrfx_gpiote_out_toggle(DAC_SPI_SS_PIN);
}

/*
 * This test slightly differs from the previous one.
 * in timeout (compare), ppi toggles cs pin once and callback toggles it back.
 * a negative pulse should be observed on cs pin, and the width is the callback delay
 * roughly 8 micro seconds, which is LARGE for mainstream microcontrollers.
 */
void test2b(void)
{
    uint32_t err, event_addr, task_addr;
    
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);

    // cycle timer init
    nrf_drv_timer_config_t cycle_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    
    err = nrf_drv_timer_init(&m_seg_timer, &cycle_cfg, test2b_cycle_timer_callback);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_extended_compare(&m_seg_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   1000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
                                   
    nrf_ppi_channel_t ppic;
    
    err = nrfx_ppi_channel_alloc(&ppic);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_seg_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppic, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppic);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_seg_timer);
}
