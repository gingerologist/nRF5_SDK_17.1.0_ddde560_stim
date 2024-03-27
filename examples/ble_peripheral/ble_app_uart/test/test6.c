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

static void test6_start(void)
{
    uint32_t err;
    
    // spi start
    static uint8_t wt[8] = { 0x90, 0x00, 0x90, 0x00, 0x90, 0x00, 0x90, 0x00 }; // set wtm mode

    static nrf_drv_spi_xfer_desc_t xfer = {
        .p_tx_buffer = wt,
        .tx_length = 2
    };

    uint32_t flags =
        NRF_DRV_SPI_FLAG_HOLD_XFER |
        // NRF_DRV_SPI_FLAG_TX_POSTINC |
        NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER |
        NRF_DRV_SPI_FLAG_REPEATED_XFER;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, flags);
    APP_ERROR_CHECK(err);

    nrf_drv_timer_enable(&m_count_timer);
    nrf_drv_timer_enable(&m_spi_timer);    
}

//static void test6_spi_timer_callback(nrf_timer_event_t event_type, void * p_context)
//{
//    // NRF_LOG_INFO("test6 spi timer cb");
//}

//static void test6_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
//{
//    NRF_LOG_INFO("test6 count timer cb")
//    NRF_LOG_INFO("  spi timer counts to %d", nrf_drv_timer_capture(&m_spi_timer, NRF_TIMER_CC_CHANNEL3));
//    NRF_LOG_INFO("  cnt timer counts to %d", nrf_drv_timer_capture(&m_count_timer, NRF_TIMER_CC_CHANNEL3));
//}

static void test6a_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("test6a count timer cb")
    NRF_LOG_INFO("  spi timer counts to %d", nrf_drv_timer_capture(&m_spi_timer, NRF_TIMER_CC_CHANNEL3));
    NRF_LOG_INFO("  cnt timer counts to %d", nrf_drv_timer_capture(&m_count_timer, NRF_TIMER_CC_CHANNEL3));
}

static void test6b_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("test6b count timer cb")
    NRF_LOG_INFO("  spi timer counts to %d", nrf_drv_timer_capture(&m_spi_timer, NRF_TIMER_CC_CHANNEL3));
    NRF_LOG_INFO("  cnt timer counts to %d", nrf_drv_timer_capture(&m_count_timer, NRF_TIMER_CC_CHANNEL3));
}

static void test6c_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("test6c count timer cb")
    NRF_LOG_INFO("  spi timer counts to %d", nrf_drv_timer_capture(&m_spi_timer, NRF_TIMER_CC_CHANNEL3));
    NRF_LOG_INFO("  cnt timer counts to %d", nrf_drv_timer_capture(&m_count_timer, NRF_TIMER_CC_CHANNEL3));
}

static void test6_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    // NRF_LOG_INFO("test6 spi evt handler");
}

/*
 * This test is inconvenient to use. The generated waveform is one shot.
 * You can only prepare probing ss/clk pin and press RUN/STOP button on oscilloscope *WHEN* 
 * flashing the program.
 *
 * This test prints spi timer count in the last count timer callback, we usually get a value of 10
 * when spi timer is configured with channel 2 for clear, as well as c0@1, c1@7, 
 * and c1 triggers count and count stops spi timer in turn.
 * This demonstrates that count stops at non-zero value. This is problematic. This is fixed in test6b.
 *
 * Based on the previous test5, test6 adds a count timer, counts up to N (=9). Here we have a question:
 *
 * Which event should be used for cs high in each spi xfer? test5 uses spi end event. This leaves a wide gap
 * between spi clk/data end and spi high. It is possibe to add another channel in spi timer to generate
 * cs high, or, use the clearing channel to do so. But be careful of the pitfall that for the 
 * last cycle of spi timer, after the count timer stops spi timer, no compare event is issued to
 * pull-up the ss pin.
 * 
 * Acturally using spi end event to trigger cs high has the advantage that when count timer counts up to required value,
 * and stops the spi timer, since spi task is already triggered, spi_end event is not influence by the
 * the stopping of spi timer. The pull-up is guaranteed.
 *
 * The following are the outdatd comment to be removed.
 *
 * spi timer c0 starts spi. c1 for cs low, as well as counting.
 * count timer c0 is oneshot and stops spi timer.
 * count timer is started manually. no trigger.
 * spi-end triggers all cs high.
 *
 * if this two timers work together as an inside loop. there should be a (repeated) trigger in outside loop to
 * start both. supposedly. This should be verified in next test.

 * alternatively. use spi timer c2 (rewind) to trigger cs high and to count, in this case c0 must be at least 4.
 */
void test6a(void)
{
    uint32_t err;

    ss_pin_init();

    // spi init
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test6_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);

    spi_timer_init(NULL);
    spi_timer_compare3(1, 7, 50);
    
    count_timer_init(test6a_count_timer_callback);

    // count timer counts up to 9 and stop
    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   9,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);

    spi_timer_c0_trigger_spi_task();
    spi_timer_c1_trigger_ss_and_count();
    count_timer_c0_stop_spi_timer();
    spi_end_trigger_ss();
    
    test6_start();
}

/*
 * In test6b spi timer is cleared by spi_end event, instead of channel 2.
 * this guarantees spi timer is cleared after count timer stops spi timer.
 */
void test6b(void)
{
    uint32_t err;
    
    ss_pin_init();

    // spi init
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test6_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);

    spi_timer_init(NULL);
    spi_timer_compare2(1, 7);
    
    count_timer_init(test6b_count_timer_callback);

    // count timer counts up to 9 and stop
    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   9,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);

    spi_timer_c0_trigger_spi_task();
    spi_timer_c1_trigger_ss_and_count();
    count_timer_c0_stop_spi_timer();
    spi_end_trigger_ss_and_clear_spi_timer();
    
    test6_start();
}

/*
 * In test6b spi timer is cleared by spi_end event, instead of channel 2.
 * this guarantees spi timer is cleared after count timer stops spi timer.
 */
void test6c(void)
{
    uint32_t err;
    
    ss_pin_init();

    // spi init
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test6_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);

    spi_timer_init(NULL);
    spi_timer_compare2(1, 7);

    count_timer_init(test6c_count_timer_callback);

    // count timer counts up to 9 and stop
    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   9,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK | NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);

    spi_timer_c0_trigger_spi_task();
    spi_timer_c1_trigger_ss_and_count();
    count_timer_c0_stop_spi_timer();
    spi_end_trigger_ss_and_clear_spi_timer();
    
    test6_start();
}

