#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_ppi.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "../howland.h"
#include "test.h"

static void test9a_spi_xfer()
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

static void test9a_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    // test9a_one_cycle_start();
    NRF_LOG_INFO("count timer finished");
}

static void test9a_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("cycle timer fired");

    test9a_spi_xfer();

    nrf_drv_timer_clear(&m_count_timer);
    nrf_drv_timer_resume(&m_count_timer);
    nrf_drv_timer_clear(&m_burst_timer);
    nrf_drv_timer_resume(&m_burst_timer);
}

/*
 * test9a use the same mode as test8a. trying to send a train of words. For merely testing purpose, we choose
 * to update G/VOUTC/TP9 in 8 words. forming a triangle/saw wave?
 *
 * init power down all but G
 * set wtm mode
 * send packet train (endless? almost)
 */
void test9a(void)
{
    uint32_t err, event_addr, task_addr;

    // gpio init ss pin
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);

    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);

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

    // spi init
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test9_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);

    // cycle timer init
    nrf_drv_timer_config_t cycle_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err = nrf_drv_timer_init(&m_cycle_timer, &cycle_cfg, test9a_cycle_timer_callback);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_extended_compare(&m_cycle_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   4000 * 1000,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    // burst timer init
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    err = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, NULL /*test8_burst_timer_callback */); // TODO remove cb?
    APP_ERROR_CHECK(err);

    // trigger spi
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL0,
                          1, // 100,
                          false);

    // trigger ss
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          7, // 100,
                          false);
    // rewind, 100Hz
    nrf_drv_timer_extended_compare(&m_burst_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   50, // 10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);

    // count timer init
    nrf_drv_timer_config_t count_cfg =
    {
        .frequency          = (nrf_timer_frequency_t)NRF_TIMER_FREQ_16MHz,
        .bit_width          = (nrf_timer_bit_width_t)NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH, // sufficient large for test
        .mode               = (nrf_timer_mode_t)1,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
    };

    err = nrf_drv_timer_init(&m_count_timer, &count_cfg, test9a_count_timer_callback);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   8,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);

    // ppis
    nrf_ppi_channel_t ppic_timc0;   // starts spi and count
    nrf_ppi_channel_t ppic_timc1;   // assert ss
    nrf_ppi_channel_t ppic_cntc0;
    nrf_ppi_channel_t ppic_spi_end;

    // timc0 trigger spi
    err = nrfx_ppi_channel_alloc(&ppic_timc0);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);

    err = nrfx_ppi_channel_assign(ppic_timc0, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    // as well as count
//    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
//    err = nrfx_ppi_channel_fork_assign(ppic_timc0, task_addr);
//    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppic_timc0);
    APP_ERROR_CHECK(err);

    // timc1 triggers ss, and counting
    err = nrfx_ppi_channel_alloc(&ppic_timc1);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL1);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppic_timc1, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
    err = nrfx_ppi_channel_fork_assign(ppic_timc1, task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppic_timc1);
    APP_ERROR_CHECK(err);

    // cntc0 stops burst timer
    err = nrfx_ppi_channel_alloc(&ppic_cntc0);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_count_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_timer_task_address_get(&m_burst_timer, NRF_TIMER_TASK_STOP);
    err = nrfx_ppi_channel_assign(ppic_cntc0, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    err =nrfx_ppi_channel_enable(ppic_cntc0);
    APP_ERROR_CHECK(err);

    // spi end triggers cs toggle
    err = nrfx_ppi_channel_alloc(&ppic_spi_end);
    APP_ERROR_CHECK(err);

    event_addr = nrf_drv_spi_end_event_get(&m_dac_spi);
    // event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL2);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppic_spi_end, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
//    task_addr = nrfx_timer_task_address_get(&m_burst_timer, NRF_TIMER_TASK_CLEAR);
//    err = nrfx_ppi_channel_fork_assign(ppic_spi_end, task_addr);
//    APP_ERROR_CHECK(err);    

//    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
//    err = nrfx_ppi_channel_fork_assign(ppic_spi_end, task_addr);
//    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppic_spi_end);
    APP_ERROR_CHECK(err);

    test9a_spi_xfer();

    nrf_drv_timer_enable(&m_count_timer);
    nrf_drv_timer_enable(&m_burst_timer);
    nrf_drv_timer_enable(&m_cycle_timer);
}
