#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_ppi.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "howland.h"
#include "timing.h"

void ss_pin_init(void)
{
    uint32_t err;

    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);    
}

void spi_timer_init(nrfx_timer_event_handler_t cb)
{
    uint32_t err;
    // spi timer init
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    err = nrf_drv_timer_init(&m_spi_timer, &timer_cfg, cb);
    APP_ERROR_CHECK(err);    
}

void spi_timer_compare2(uint32_t c0_val, uint32_t c1_val)
{
    // spi timer channel 0 compare 1/16 micro seconds
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL0,
                          c0_val, // 100,
                          false);

    // spi timer channel 1 compare 7/16 micro seconds
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          c0_val, // 100,
                          false);
}

void spi_timer_compare3(uint32_t c0_val, uint32_t c1_val, uint32_t c2_val)
{
    // spi timer channel 0 compare 1/16 micro seconds
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL0,
                          c0_val, // 100,
                          false);

    // spi timer channel 1 compare 7/16 micro seconds
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          c1_val, // 100,
                          false);
    
    // spi timer channel 2 compare 50/16 micro seconds to generate period
    nrf_drv_timer_extended_compare(&m_spi_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   c2_val, // 10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);    
}

void count_timer_init(nrfx_timer_event_handler_t cb)
{
    uint32_t err;
    
    // count timer init
    nrf_drv_timer_config_t count_cfg =
    {
        .frequency          = (nrf_timer_frequency_t)NRF_TIMER_FREQ_16MHz,
        .bit_width          = (nrf_timer_bit_width_t)NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH, // sufficient large for test
        .mode               = (nrf_timer_mode_t)1,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    };

    err = nrf_drv_timer_init(&m_count_timer, &count_cfg, cb);
    APP_ERROR_CHECK(err);    
}

void count_timer_compare(uint32_t c0_val)
{
    // count timer counts up to 9 and stop
    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   c0_val,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);    
}

void spi_timer_c0_trigger_spi_task(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);    
}

void spi_timer_c0_trigger_spi_task_and_count(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
    err = nrfx_ppi_channel_fork_assign(ppi, task_addr);
    APP_ERROR_CHECK(err);    
    
    err = nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);    
}

void spi_timer_c1_trigger_ss_and_count(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;    
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL1);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
    err = nrfx_ppi_channel_fork_assign(ppi, task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);    
}

void count_timer_c0_stop_spi_timer(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;    
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_count_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_timer_task_address_get(&m_spi_timer, NRF_TIMER_TASK_STOP);
    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    err =nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);    
}

void spi_end_trigger_ss(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;    
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrf_drv_spi_end_event_get(&m_dac_spi);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err =nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);      
}

void spi_end_trigger_ss_and_clear_spi_timer(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;    
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrf_drv_spi_end_event_get(&m_dac_spi);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    task_addr = nrfx_timer_task_address_get(&m_spi_timer, NRF_TIMER_TASK_CLEAR);
    err = nrfx_ppi_channel_fork_assign(ppi, task_addr);
    APP_ERROR_CHECK(err);     
    
    err =nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);       
}

void cycle_timer_c0_trigger_spi_timer(void)
{
    uint32_t err, event_addr, task_addr;
    nrf_ppi_channel_t ppi;    
    
    err = nrfx_ppi_channel_alloc(&ppi);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_segment_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_timer_task_address_get(&m_spi_timer, NRF_TIMER_TASK_START);

    err = nrfx_ppi_channel_assign(ppi, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err =nrfx_ppi_channel_enable(ppi);
    APP_ERROR_CHECK(err);           
}
