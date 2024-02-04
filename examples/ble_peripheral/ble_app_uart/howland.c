#include "nrf_spi_mngr.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "nrfx_ppi.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "howland.h"

/**
 * Burst timer generates rising and falling edge ppi, as well as interrupt (for reconfigure)
 */
#define COUNT_TIMER_ID              1       // counter
#define BURST_TIMER_ID              2       
#define CYCLE_TIMER_ID              3       

#define DAC_SPI_ID                  1
#define ASW_SPI_ID                  2

/**
 * freertos task
 */
static TaskHandle_t m_thread = NULL;

/**
 * freertos queue
 */
#define INCOMMING_QUEUE_DEPTH               8

static QueueHandle_t m_incomming_idle       = NULL;
static QueueHandle_t m_incomming_pending    = NULL;
static ble_incomming_message_t m_message_pool[INCOMMING_QUEUE_DEPTH] = { 0 };

static void incomming_queue_init(void)
{
    ble_incomming_message_t * p_msg;
    
    m_incomming_idle = xQueueCreate(INCOMMING_QUEUE_DEPTH, sizeof(ble_incomming_message_t *));
    APP_ERROR_CHECK_BOOL(m_incomming_idle != NULL);
    m_incomming_pending = xQueueCreate(INCOMMING_QUEUE_DEPTH, sizeof(ble_incomming_message_t *));
    APP_ERROR_CHECK_BOOL(m_incomming_idle != NULL);
    
    for (int i = 0; i < INCOMMING_QUEUE_DEPTH; i++)
    {
        p_msg = &m_message_pool[i];
        BaseType_t result = xQueueSend(m_incomming_idle, &p_msg, 0);
        APP_ERROR_CHECK_BOOL(result == pdTRUE);
    }    
}

/**
 * module wide statics
 */
// m_msg is both state and message
static ble_incomming_message_t m_msg = {
    .c              = 0,
    .timeout        = 0,
    .freq           = 100,
    .num_of_pulses  = 1,
    .pulse_width    = 50,
    .pulse_interval = 0,
    .hold           = 0,
    .recycle_ratio  = 4,
};

/**
 * SPI and DAC088S085CIMT
 */
NRF_SPI_MNGR_DEF(m_dac_spi_mngr, 16, DAC_SPI_ID);

static nrf_drv_spi_config_t const m_dac_spi_config =
{
    .sck_pin = 12,
    .mosi_pin = 1,
    .miso_pin = NRF_DRV_SPI_PIN_NOT_USED,
    .ss_pin = 14,
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .orc = 0xFF,
    .frequency = NRF_DRV_SPI_FREQ_8M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

/**
 * WRM Write Register Mode, registered are written without updating output
 * WTM Write Through Mode, output is updated immediately after the register is written
 * defaults to WRM mode after power up
 */
typedef enum {
    DAC_WRM_MODE = 0,
    DAC_WTM_MODE
} dac_update_mode_t;

// mode must be either DAC_WRM_MODE or DAC_WTM_MODE
static void set_dac_update_mode(dac_update_mode_t mode)
{
    static uint16_t reg;
    static nrf_spi_mngr_transfer_t const xfers[] = { NRF_SPI_MNGR_TRANSFER(&reg, 2, NULL, 0) };
    static dac_update_mode_t _mode = DAC_WRM_MODE; // power on default
    if (mode == _mode) return;
    
    reg = mode == DAC_WRM_MODE ? 0x8000 : 0x9000;
    
    nrf_spi_mngr_perform(&m_dac_spi_mngr, &m_dac_spi_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
    
    _mode = mode;
}

// see 
static uint16_t dac_wrm_mode = 0x8000;
static uint16_t dac_wtm_mode = 0x9000;
static uint16_t dac_cmd_update_select = 0xA000;
static uint16_t dac_cmd_update_all = 0xA0FF;
static uint16_t dac_cmd_chan_a_write = 0xB000;
static uint16_t dac_cmd_broadcast = 0xC000;

static nrf_spi_mngr_transfer_t dac_wrm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wrm_mode, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t dac_wtm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wtm_mode, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t dac_cmd_update_select_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_update_select, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t dac_cmd_chan_a_write_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_chan_a_write, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t dac_cmd_broadcast_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_broadcast, 2, NULL, 0) };

static uint16_t dac_reg_init[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };
static uint16_t dac_reg_pulse[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };
static uint16_t dac_reg_recycle[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };

// this function update 8 channels simultaneously
// there are two ways to do so
// 1. set wrm mode, update 8 channels, special command 1
// 2. set wrm mode, update 7 channles, special command 2
// here we use the second method.
static void dac_update_simult(uint16_t data[8])
{
    (void)dac_wrm_mode;
    (void)dac_wtm_mode;
    (void)dac_cmd_update_select;
    (void)dac_cmd_chan_a_write;
    (void)dac_wrm_mode_xfer;
    (void)dac_wtm_mode_xfer;
    (void)dac_cmd_update_select_xfer;
    (void)dac_cmd_chan_a_write_xfer;
    (void)dac_reg_init;
    (void)dac_reg_pulse;
    (void)dac_reg_recycle;
    (void)dac_update_simult;
    
    set_dac_update_mode(DAC_WRM_MODE);
    
    uint16_t cha = data[0] | 0xb000;
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&data[1], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[2], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[3], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[4], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[5], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[6], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[7], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&cha,     2, NULL, 0)
    };
    
//    nrf_spi_mngr_transaction_t trans =
//    {
//        .begin_callback = NULL,
//        .end_callback = NULL,
//        .p_user_data = NULL,
//        .p_transfers = xfers,
//        .number_of_transfers = sizeof(xfers) / sizeof(xfers[0]),
//        .p_required_spi_cfg = NULL,
//    };
    
    nrf_spi_mngr_perform(&m_dac_spi_mngr, &m_dac_spi_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
//    nrf_spi_mngr_schedule(&m_dac_spi_mngr, &trans);
}

static void dac_update_pulse()
{
    uint32_t err_code;
    set_dac_update_mode(DAC_WRM_MODE);
    
    static nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[0], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[1], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[2], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[3], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[4], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[5], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[6], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_reg_pulse[7], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&dac_cmd_update_all, 2, NULL, 0),
    };
    
    static nrf_spi_mngr_transaction_t trans =
    {
        .begin_callback = NULL,
        .end_callback = NULL,
        .p_user_data = NULL,
        .p_transfers = xfers,
        .number_of_transfers = sizeof(xfers) / sizeof(xfers[0]),
        .p_required_spi_cfg = NULL,
    };
    
    err_code = nrf_spi_mngr_schedule(&m_dac_spi_mngr, &trans);
    APP_ERROR_CHECK(err_code);
}

static void dac_update_recycle()
{
    
}

static void dac_update_sequent(uint16_t data[8])
{
    (void)dac_update_sequent;
    
    set_dac_update_mode(DAC_WTM_MODE);
    
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&data[0], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[1], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[2], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[3], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[4], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[5], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[6], 2, NULL, 0),
        NRF_SPI_MNGR_TRANSFER(&data[7], 2, NULL, 0)
    };
    
    nrf_spi_mngr_perform(&m_dac_spi_mngr, &m_dac_spi_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);    
}

static void dac_update_same(uint8_t val)
{
    (void)dac_update_same;

    uint16_t data = (((uint16_t)val) << 4) | 0xC000;
    
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&data, 2, NULL, 0),
    };
    
    nrf_spi_mngr_perform(&m_dac_spi_mngr, &m_dac_spi_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
}

/***
 * SPI and Analog Switch
 * 
 * hardware signal mapping
 * A -> S6 (1 << 5) 0x20
 * B -> S5 (1 << 4) 0x10
 * C -> S4 (1 << 3) 0x08
 * D -> S3 (1 << 2) 0x04
 * E -> S8 (1 << 7) 0x80
 * F -> S7 (1 << 6) 0x40
 * G -> S2 (1 << 1) 0x02
 * H -> S1 (1 << 0) 0x01
 */

// spi manager
NRF_SPI_MNGR_DEF(m_asw_spi_mngr, 16, ASW_SPI_ID);

// spi config
static nrf_drv_spi_config_t const m_asw_spi_config =
{
    .sck_pin        = 6,                            // P0.06 SW_CLK_A
    .mosi_pin       = 8,                            // P0.08 SW_DIN_A
    .miso_pin       = NRF_DRV_SPI_PIN_NOT_USED,
    .ss_pin         = 5,                            // P0.05 SW_SYNC_A
    .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
    .orc            = 0xFF,
    .frequency      = NRF_DRV_SPI_FREQ_8M,
    .mode           = NRF_DRV_SPI_MODE_2,           // CPOL = 1, CLK idle high
                                                    // CPHA = 0, data sampled on falling edge
                                                    // see adg1414.pdf, figure 2, also serial peripheral interface on wiki
    .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

static void asw_on(void)
{
    const static uint8_t mask[8] = { 1 << 5, 1 << 4, 1 << 3, 1 << 2, 1 << 7, 1 << 6, 1 << 1, 1 << 0 };
    uint8_t cfg = 0;
    
    cfg |= m_msg.current[0] ? mask[0] : 0;    // A
    cfg |= m_msg.current[1] ? mask[1] : 0;    // B
    cfg |= m_msg.current[2] ? mask[2] : 0;    // C
    cfg |= m_msg.current[3] ? mask[3] : 0;    // D
    cfg |= m_msg.current[4] ? mask[4] : 0;    // E
    cfg |= m_msg.current[5] ? mask[5] : 0;    // F
    cfg |= m_msg.current[6] ? mask[6] : 0;    // G
    cfg |= m_msg.current[7] ? mask[7] : 0;    // H
    
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&cfg, 1, NULL, 0),
    };
    
    nrf_spi_mngr_perform(&m_asw_spi_mngr, NULL, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);    
}

static void asw_off(void)
{
    uint8_t cfg = 0;
    
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&cfg, 1, NULL, 0),
    };
    
    nrf_spi_mngr_perform(&m_asw_spi_mngr, NULL, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);        
}

/**
 * Timer
 */
static const nrf_drv_timer_t m_count_timer = NRF_DRV_TIMER_INSTANCE(COUNT_TIMER_ID);
static const nrf_drv_timer_t m_cycle_timer = NRF_DRV_TIMER_INSTANCE(CYCLE_TIMER_ID);
static const nrf_drv_timer_t m_burst_timer = NRF_DRV_TIMER_INSTANCE(BURST_TIMER_ID);

static void test1_burst_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("burst channel 0 fired");
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
            NRF_LOG_INFO("cycle channel 0 fired");
            break;
        default:
            break;
    }    
}


static void test1_timer_init(void)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_cycle_timer, &timer_cfg, test1_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test1_burst_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);    
}

/**
 *  In test1, cycle timer fires once. Both ppi and interrupt handler are configured.
 *  ppi start burst timer, which also fires once, printing something in interrupt handler.
 *  We expect both print works, indicating that ppi and interrupt handler could work simultaneously.
 */
static void test1_run(void)
{
    nrfx_err_t err;
    nrf_ppi_channel_t ppi_channel;
    
    test1_timer_init();
    
    nrf_drv_timer_extended_compare(&m_cycle_timer, 
                                   NRF_TIMER_CC_CHANNEL0, 
                                   3 * 1000 * 1000, 
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);

    nrf_drv_timer_extended_compare(&m_burst_timer, 
                                   NRF_TIMER_CC_CHANNEL0, 
                                   3 * 1000 * 1000, 
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);    
    
    err = nrfx_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err);

    uint32_t cycle_timer_event_address = nrfx_timer_compare_event_address_get(&m_cycle_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t burst_timer_task_address = nrfx_timer_task_address_get(&m_burst_timer, NRF_TIMER_TASK_START);
    
    err = nrfx_ppi_channel_assign(ppi_channel, cycle_timer_event_address, burst_timer_task_address);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_cycle_timer);
    NRF_LOG_INFO("test1 started");
}

static int test2_count = 0;

static void test2_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
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

static void test2_burst_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("burst 0 -");
            break;
        default:
            NRF_LOG_INFO("burst fired");
            break;
    }    
}

static void test2_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("cycle 0 +");
            nrf_drv_timer_clear(&m_burst_timer);
            break;
        case NRF_TIMER_EVENT_COMPARE1:
            NRF_LOG_INFO("cycle channel 1 fired");
            break;        
        default:
            break;
    }    
}

static void test2_timer_init(void)
{
    uint32_t err_code;
    
    nrf_drv_timer_config_t count_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    count_cfg.mode = (nrf_timer_mode_t)1;   // 1 for counter mode, 
    err_code = nrf_drv_timer_init(&m_count_timer, &count_cfg, test2_count_timer_callback);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_cycle_timer, &timer_cfg, test2_cycle_timer_callback);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test2_burst_timer_callback);
    APP_ERROR_CHECK(err_code);    
}

/**
 * In test2
 * burst timer is one-shot, log a number in interrupt.
 * cycle timer compares to 1s, ppi to increment counter and to start burst timer as well .
 * counter set to compare 5.
 * 
 * don't know why burst timer fires only once. But count timer does work.
 */
static void test2_run(void)
{
    nrfx_err_t err;
    
    test2_timer_init();
    
    // count timer channel 0 count to 5
    nrf_drv_timer_extended_compare(&m_count_timer, NRF_TIMER_CC_CHANNEL0, 5, NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    
    // short-lived one-shot, print in isr, triggered by cycle timer 
    nrf_drv_timer_extended_compare(&m_burst_timer, NRF_TIMER_CC_CHANNEL0, 50, NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    // nrf_drv_timer_compare(&m_burst_timer, NRF_TIMER_CC_CHANNEL0, 1000, true);
    // nrf_drv_timer_enable(&m_burst_timer);
    
    // cycle timer repeats in seconds.
    nrf_drv_timer_extended_compare(&m_cycle_timer, NRF_TIMER_CC_CHANNEL0, 2 * 1000 * 1000, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    
    // cycle start burst
    nrf_ppi_channel_t cycle_start_burst;
    err = nrfx_ppi_channel_alloc(&cycle_start_burst);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_assign(cycle_start_burst, 
                                  nrfx_timer_compare_event_address_get(&m_cycle_timer, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_burst_timer, NRF_TIMER_TASK_START));
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(cycle_start_burst);
    APP_ERROR_CHECK(err);
    
    // cycle increment count
    nrf_ppi_channel_t cycle_increment_count;
    err = nrfx_ppi_channel_alloc(&cycle_increment_count);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_assign(cycle_increment_count,
                                  nrfx_timer_compare_event_address_get(&m_cycle_timer, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(cycle_increment_count);
    APP_ERROR_CHECK(err);                              
                                                             
    // count stop cycle
    nrf_ppi_channel_t count_stop_cycle;
    err = nrfx_ppi_channel_alloc(&count_stop_cycle);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_assign(count_stop_cycle,
                                  nrfx_timer_compare_event_address_get(&m_count_timer, NRF_TIMER_CC_CHANNEL0),
                                  nrfx_timer_task_address_get(&m_cycle_timer, NRF_TIMER_TASK_STOP));
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(count_stop_cycle);
    APP_ERROR_CHECK(err);                              
    
    nrf_drv_timer_enable(&m_count_timer);
    nrf_drv_timer_enable(&m_cycle_timer);
    
    NRF_LOG_INFO("test2 started 2");
}

static void burst_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
}

static void cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("channel 0 fired");
            dac_update_pulse();
            // NRF_LOG_INFO("inside isr %d", INSIDE_ISR);
            break;
        case NRF_TIMER_EVENT_COMPARE1:
            NRF_LOG_INFO("channel 1 fired");
            // dac_update_simult(dac_reg_recycle);
            break;
        case NRF_TIMER_EVENT_COMPARE2:  // won't fire
            NRF_LOG_INFO("channel 2 fired");
            break;
        default:
            break;
    }
}

static void timer_init(void)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_cycle_timer, &timer_cfg, cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, burst_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
}

/**
 * Timer 0 is reserved by SoftDevice
 * Timer 1 or 2 has 4 channels
 * Timer 3 or 4 has 6 channels
 *
 * channel 0, pulse on
 * channel 1, recycle on
 * channel 2, recycle off
 * channel 3, rewind (period end)
 */

static void stim_prepare(void)
{
//    uint32_t slow_factor = 5000;
//    uint32_t init_delay = 1;
//    
//    // prepare dac reg
//    for (int i = 0; i < 8; i++)
//    {
//        int c = m_msg.current[i];
//        
//        if (c == 0)
//        {
//            dac_reg_pulse[i] = 0x0800;
//            dac_reg_recycle[i] = 0x0800;
//            continue;
//        }
//        
//        dac_reg_pulse[i] = 2048 + c * 2047 / 15;
//        dac_reg_recycle[i] = 2048 - c * 2047 / 15 / m_msg.recycle_ratio;
//    }
//    
//    // fire interrupt at the beginning of pulse, 0 does NOT work, at least 1
//    nrf_drv_timer_compare(&m_cycle_timer, NRF_TIMER_CC_CHANNEL0, init_delay * slow_factor, true);
//    // fire interrupt at the beginning of recycle
//    nrf_drv_timer_compare(&m_cycle_timer, NRF_TIMER_CC_CHANNEL1, (m_msg.pulse_width + init_delay) * slow_factor, true);
//    // rewind
//    nrf_drv_timer_extended_compare(&m_cycle_timer, NRF_TIMER_CC_CHANNEL2, (m_msg.pulse_width + m_msg.recycle_ratio - init_delay) * slow_factor, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, false);    
    
    NRF_LOG_INFO("howland stim prepared");
}

static void stim_start(void)
{
//    asw_on();
//    nrf_drv_timer_enable(&m_cycle_timer);
    
    NRF_LOG_INFO("howland stim started");
}

static void stim_stop(void)
{
//    nrf_drv_timer_disable(&m_cycle_timer);
//    asw_off();
    
    NRF_LOG_INFO("howland stim stopped");
}

/**
 * Howland
 */
static void howland_task(void * pvParameters)
{
    // test1_run();
    test2_run();
    vTaskDelay(portMAX_DELAY);
//  ret_code_t ret;
    
    ble_incomming_message_t * p_msg;
    uint16_t msg_length = sizeof(m_msg);
    
    bool stim_started = false;
    
    incomming_queue_init();
    
    timer_init();
    
    nrf_spi_mngr_init(&m_dac_spi_mngr, &m_dac_spi_config);
    // dac_update_simult(dac_reg_init);
    dac_update_same(0x80);
    
    nrf_spi_mngr_init(&m_asw_spi_mngr, &m_asw_spi_config);
    
    while (xQueueReceive(m_incomming_pending, &p_msg, portMAX_DELAY))
    {
        switch (p_msg->c) 
        {
            case 0: // READ
                NRF_LOG_INFO("read from app");
                ble_nus_send((uint8_t *)&m_msg, &msg_length);
                break;
            case 1: // STOP
                if (stim_started)
                {
                    stim_stop();
                    m_msg.c = 0;
                    stim_started = false;
                }
                ble_nus_send((uint8_t *)&m_msg, &msg_length);
                break;
            case 2: { // START // TODO check p_msg and reject
                if (stim_started)
                {
                    stim_stop();    
                }
                
                // TODO 
//                m_msg = *p_msg;
//                m_msg.c = 0xffff;
//                m_msg.timeout = 0xffff;
//                m_msg.num_of_pulses = 1; 
//            
//                m_msg.pulse_width = (m_msg.pulse_width < 20) ? 20 : m_msg.pulse_width;
//                m_msg.pulse_width = (m_msg.pulse_width > 50 * 1000) ? 50 * 1000 : m_msg.pulse_width;
//            
//                m_msg.pulse_interval = 0;
//                m_msg.mid = 0;
//                // m_msg.freq = 0;
//            
//                m_msg.recycle = (m_msg.recycle < m_msg.pulse_width) ? m_msg.pulse_width : m_msg.recycle;
//                m_msg.recycle = (m_msg.recycle > 10 * m_msg.pulse_width) ? 10 * m_msg.pulse_width : m_msg.recycle;

//                stim_prepare();
//                stim_start();
//                stim_started = true;
                break;
            }
            default:
                break;
        }
        
        xQueueSend(m_incomming_idle, &p_msg, portMAX_DELAY);
        // TODO LOG error
    }
}

void howland_freertos_init(void)
{
    BaseType_t xReturned;
    
    xReturned = xTaskCreate(howland_task,
                            "howl",
                            1024,   // stack size in word
                            NULL,
                            1,      // priority
                            &m_thread);

    if (xReturned != pdPASS)
    {
        NRF_LOG_INFO("no mem for howland task.");
    }
    else
    {
        NRF_LOG_INFO("howland task ready.");
    }    
}

ble_incomming_message_t * ble_nus_alloc(void)
{
    ble_incomming_message_t * p_msg;
    
    NRF_LOG_INFO("idle q has %d messages", uxQueueMessagesWaiting(m_incomming_idle));
    
    if (pdTRUE == xQueueReceive(m_incomming_idle, &p_msg, 0))
    {
        return p_msg;
    }
    else
    {
        return NULL;
    }
}

ble_incomming_message_t * ble_nus_alloc_from_isr(void)
{
    ble_incomming_message_t * p_msg;
    if (pdTRUE == xQueueReceiveFromISR(m_incomming_idle, &p_msg, 0))
    {
        return p_msg;
    }
    else
    {
        return NULL;
    }
}

void ble_nus_recv(ble_incomming_message_t * p_message)
{
    if (p_message == NULL) return;
   
    // TODO log error
    xQueueSend(m_incomming_pending, &p_message, 0);
}

void ble_nus_recv_from_isr(ble_incomming_message_t * p_message)
{
    if (p_message == NULL) return;
    
    // TODO log error
    xQueueSendFromISR(m_incomming_pending, &p_message, 0);
}
