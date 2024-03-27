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

static void test7a_spi_xfer()
{
    uint32_t err;

    // spi start
    static uint8_t data[16] = {
        0x60, 0x00,
        0x64, 0x00,
        0x68, 0x00,
        0x6c, 0x00,
        0x6f, 0xf0,
        0x6c, 0x00,
        0x68, 0x00,
        0x64, 0x00,
    };
    
    static nrf_drv_spi_xfer_desc_t xfer = {
        .p_tx_buffer = data,
        .tx_length = 2
    };

    const uint32_t flags =
        NRF_DRV_SPI_FLAG_HOLD_XFER |
        NRF_DRV_SPI_FLAG_TX_POSTINC |
        NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER |
        NRF_DRV_SPI_FLAG_REPEATED_XFER;

    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, flags);
    APP_ERROR_CHECK(err);
}

// TODO is this necessary?
static void test9_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    // NRF_LOG_INFO("test8 spi evt handler");
}

static void test7a_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    // test7a_one_cycle_start();
    NRF_LOG_INFO("count timer finished");
}

static void test7a_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("cycle timer fired");

    test7a_spi_xfer();
    nrf_drv_timer_resume(&m_spi_timer);
}

/*
 * test7a use the same mode as test8a. trying to send a train of words. For merely testing purpose, we choose
 * to update G/VOUTC/TP9 in 8 words. forming a triangle/saw wave?
 *
 * init power down all but G
 * set wtm mode
 * send packet train (endless? almost)
 */
void test7a(void)
{
    uint32_t err;

    // gpio init ss pin
    ss_pin_init();
    
    static uint8_t pd[2] = { 0xD0, 0xBF };  // power down all but g
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode

    // init sensor in blocking mode
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);
    nrf_drv_spi_transfer(&m_dac_spi, pd, 2, NULL, 0);
    nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);

    nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);
    nrf_drv_spi_transfer(&m_dac_spi, wt, 2, NULL, 0);
    nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);

    nrf_drv_spi_uninit(&m_dac_spi);

    // spi reinit (non-blocking mode)
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test9_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);

    // cycle timer init
    nrf_drv_timer_config_t cycle_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err = nrf_drv_timer_init(&m_segment_timer, &cycle_cfg, test7a_cycle_timer_callback);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_extended_compare(&m_segment_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   4000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    spi_timer_init(NULL);
    spi_timer_compare2(1, 7);
    
    count_timer_init(test7a_count_timer_callback);
    count_timer_compare(8);

    spi_timer_c0_trigger_spi_task();
    spi_timer_c1_trigger_ss_and_count();
    count_timer_c0_stop_spi_timer(); // original code fork assign STOP and CLEAR
    spi_end_trigger_ss_and_clear_spi_timer();

    test7a_spi_xfer();

    nrf_drv_timer_enable(&m_count_timer);
    nrf_drv_timer_enable(&m_spi_timer);
    nrf_drv_timer_enable(&m_segment_timer);
}
