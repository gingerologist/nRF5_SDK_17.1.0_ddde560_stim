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

static void test1_spi_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("spi timer channel 0 fired");
            break;
        default:
            break;
    }
}

static void test1_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("cycle timer channel 0 fired");
            break;
        default:
            break;
    }
}


static void test1_timer_init(void)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_seg_timer, &timer_cfg, test1_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_timer_init(&m_spi_timer, &timer_cfg, test1_spi_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
}

/**
 *  test1 proves that for a timer compare event, both ppi and interupt handler can take effect.
 *
 *  In test1, cycle timer fires once. Both ppi and interrupt handler are configured.
 *  ppi start spi timer, which also fires once, printing something in interrupt handler. (print once, not periodically)
 *  We expect both prints work, indicating that ppi and interrupt handler could work simultaneously.
 */
void test1(void)
{
    nrfx_err_t err;
    nrf_ppi_channel_t ppi_channel;

    test1_timer_init();

    nrf_drv_timer_extended_compare(&m_seg_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   3 * 1000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

    nrf_drv_timer_extended_compare(&m_spi_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   3 * 1000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

    err = nrfx_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err);

    uint32_t cycle_timer_event_address = nrfx_timer_compare_event_address_get(&m_seg_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t spi_timer_task_address = nrfx_timer_task_address_get(&m_spi_timer, NRF_TIMER_TASK_START);

    err = nrfx_ppi_channel_assign(ppi_channel, cycle_timer_event_address, spi_timer_task_address);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_enable(&m_seg_timer);
    NRF_LOG_INFO("test1 started");
}

