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

static uint8_t st_regs[216] = {
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
    0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 
};

#define TIME_1SEC  (16 * 1000 * 1000)

static uint32_t st_acctime[12] = {
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC,
    TIME_1SEC
};

static int st_index = 0;
static int st_num = 12;

static uint32_t st_total_time = 16 * 1000 * 1000 * 12;

static void test8_spi_xfer()
{
    uint32_t err;
    
    static nrf_drv_spi_xfer_desc_t xfer = {
        .p_tx_buffer = st_regs,
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

static void test8_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0)
    {
        nrf_drv_timer_compare(&m_seg_timer, 
                              NRF_TIMER_CC_CHANNEL0,
                              st_acctime[st_index],
                              true);
        st_index = (st_index + 1) % st_num;
    }
    else if (event_type == NRF_TIMER_EVENT_COMPARE1)
    {
        test8_spi_xfer();
    }
}

/*
 * test7a use the same mode as test8a. trying to send a train of words. For merely testing purpose, we choose
 * to update G/VOUTC/TP9 in 8 words. forming a triangle/saw wave?
 *
 * init power down all but G
 * set wtm mode
 * send packet train (endless? almost)
 */
void test8a(void)
{
    uint32_t err, event_addr, task_addr;

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
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    APP_ERROR_CHECK(err);

    // cycle timer init
    nrf_drv_timer_config_t cycle_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err = nrf_drv_timer_init(&m_seg_timer, &cycle_cfg, test8_cycle_timer_callback);
    APP_ERROR_CHECK(err);

    // first config, will be reconfigured in isr
    nrf_drv_timer_compare(&m_seg_timer, 
                          NRF_TIMER_CC_CHANNEL0, 
                          st_acctime[st_num - 1],  // last one
                          true);
    // 
    nrf_drv_timer_extended_compare(&m_seg_timer,
                                   NRF_TIMER_CC_CHANNEL1,
                                   st_total_time,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    // enable timer in stopped state
    spi_timer_init(NULL);
    nrf_drv_timer_enable(&m_spi_timer);
    nrf_drv_timer_pause(&m_spi_timer);
    nrf_drv_timer_clear(&m_spi_timer);
    spi_timer_compare2(1, 7);
    
    // never stopped
    count_timer_init(NULL);
    nrf_drv_timer_enable(&m_seg_counter);
    count_timer_compare(9);

    spi_timer_c0_trigger_spi_task();
    spi_timer_c1_trigger_ss_and_count();
    count_timer_c0_stop_spi_timer(); // original code fork assign STOP and CLEAR
    spi_end_trigger_ss_and_clear_spi_timer();
    cycle_timer_c0_trigger_spi_timer();

    test8_spi_xfer();
    nrf_drv_timer_enable(&m_seg_timer);
}
