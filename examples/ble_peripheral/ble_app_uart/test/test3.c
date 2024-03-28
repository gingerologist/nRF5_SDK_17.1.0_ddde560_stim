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

static void test3_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("count 0");
            break;
        default:
            break;
    }
}

static void test3_spi_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("spi 0 -");
            break;
        default:
            NRF_LOG_INFO("spi fired");
            break;
    }
}

static void test3_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("cycle 0 +");
            nrf_drv_timer_clear(&m_spi_timer);
            break;
        case NRF_TIMER_EVENT_COMPARE1:
            NRF_LOG_INFO("cycle channel 1 fired");
            break;
        default:
            break;
    }
}

/*
 * 
 */
static void test3_timer_init(void)
{
    uint32_t err_code;

    nrf_drv_timer_config_t count_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    count_cfg.mode = (nrf_timer_mode_t)1;   // 1 for counter mode,
    err_code = nrf_drv_timer_init(&m_seg_counter, &count_cfg, test3_count_timer_callback);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_seg_timer, &timer_cfg, test3_cycle_timer_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_spi_timer, &timer_cfg, test3_spi_timer_callback);
    APP_ERROR_CHECK(err_code);
}

/**
 * In test3, the lifespan of cycle timer contains 5 times of spi timer.
 *
 * spi timer is one-shot, log a number in interrupt.
 * cycle timer compares to 1s, ppi to increment counter and to start spi timer as well.
 * counter set to compare 5 and stop cycle
 */
void test3(void)
{
    nrfx_err_t err;

    test3_timer_init();

    // count timer channel 0 count to 5 then stop, with callback
    nrf_drv_timer_extended_compare(&m_seg_counter, NRF_TIMER_CC_CHANNEL0, 5, NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

    // short-lived one-shot, print in isr, triggered by cycle timer
    nrf_drv_timer_extended_compare(&m_spi_timer, NRF_TIMER_CC_CHANNEL0, 50, NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    // nrf_drv_timer_compare(&m_spi_timer, NRF_TIMER_CC_CHANNEL0, 1000, true);
    // nrf_drv_timer_enable(&m_spi_timer);

    // cycle timer repeats in seconds.
    nrf_drv_timer_extended_compare(&m_seg_timer, NRF_TIMER_CC_CHANNEL0, 2 * 1000 * 1000, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    // cycle start spi
    nrf_ppi_channel_t cycle_start_spi;
    err = nrfx_ppi_channel_alloc(&cycle_start_spi);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_assign(cycle_start_spi,
                                  nrfx_timer_compare_event_address_get(&m_seg_timer, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_spi_timer, NRF_TIMER_TASK_START));
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(cycle_start_spi);
    APP_ERROR_CHECK(err);

    // cycle increment count
    nrf_ppi_channel_t cycle_increment_count;
    err = nrfx_ppi_channel_alloc(&cycle_increment_count);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_assign(cycle_increment_count,
                                  nrfx_timer_compare_event_address_get(&m_seg_timer, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_seg_counter, NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(cycle_increment_count);
    APP_ERROR_CHECK(err);

    // count stop cycle
    nrf_ppi_channel_t count_stop_cycle;
    err = nrfx_ppi_channel_alloc(&count_stop_cycle);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_assign(count_stop_cycle,
                                  nrfx_timer_compare_event_address_get(&m_seg_counter, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_seg_timer, NRF_TIMER_TASK_STOP));
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(count_stop_cycle);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_enable(&m_seg_counter);
    nrf_drv_timer_enable(&m_seg_timer);

    NRF_LOG_INFO("test3 started 2");
}
