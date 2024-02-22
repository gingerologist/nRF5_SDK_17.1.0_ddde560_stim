#include "nrf_drv_spi.h"
#include "nrf_spi_mngr.h"
#include "nrf_drv_timer.h"
// #include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"
// #include "nrfx_spim.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "howland.h"

// faild to try spim driver
// #define DAC_USE_SPI_MNGR

/**
 * DAC088S085 Commands
 * 
 * 0xF x 8b     Power Down 2.5kOhm output
 * 0xE x 8b     Power Down 100kOhm output
 * 0xD x 8b     Power Down HighZ
 * 0xC 8d x     broadcast
 * 0xB 8d x     Channel A Write
 * 0xA x 8b     Update select
 * 0x9 x  x     set wtm mode
 * 0x8 x  x     set wrm mode (default)
 * 0x7 8d x     set channel H
 * 0x6 8d x
 *  .
 *  .
 * 0x0 8d x     set channel A
 
 * also, due to layout, pin name and netname inconsistent
 * 
 * PIN NAME     NET NAME        PORT NAME
 * A            VOUTE           PORT_F
 * B            VOUTF           PORT_E    
 * C            VOUTG (TP1)     PORT_B (TP2)
 * D            VOUTH (TP3)     PORT_A (TP4)
 * E            VOUTA           PORT_G
 * F            VOUTB           PORT_H
 * G            VOUTC (TP9)     PORT_C (TP8)
 * H            VOUTD (TP7)     PORT_D (TP6)
 *
 * TPs (at top edge/side) are numbered 1234 5 6789 from left to right, 5 is VMID (2.5V)
 * 
 */ 

/**
 * Burst timer generates rising and falling edge ppi, as well as interrupt (for reconfigure)
 */
#define COUNT_TIMER_ID              1       // counter
#define BURST_TIMER_ID              2       
#define CYCLE_TIMER_ID              3       

#define DAC_SPI_INSTANCE            1
#define ASW_SPI_ID                  2

#define DAC_SPI_SS_PIN              14
#define DAC_SPI_CK_PIN              12
#define DAC_SPI_SI_PIN              1

// nrf_drv_spi context
typedef struct dac_spi_ctx {
    int value;
    bool bypass;
} dac_spi_ctx_t;

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
#ifdef DAC_USE_SPI_MNGR
NRF_SPI_MNGR_DEF(m_dac_spi_mngr, 16, DAC_SPI_INSTANCE);
#else
static const nrf_drv_spi_t m_dac_spi = NRF_DRV_SPI_INSTANCE(DAC_SPI_INSTANCE);  /**< SPI instance. */
#endif

static nrf_drv_spi_config_t const m_dac_spi_config =
{
    .sck_pin        = DAC_SPI_CK_PIN,
    .mosi_pin       = DAC_SPI_SI_PIN,
    .miso_pin       = NRF_DRV_SPI_PIN_NOT_USED,
    .ss_pin         = DAC_SPI_SS_PIN,
    .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
    .orc            = 0xFF,
    .frequency      = NRF_DRV_SPI_FREQ_8M,
    .mode           = NRF_DRV_SPI_MODE_1,
    .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

static nrf_drv_spi_config_t const m_dac_spi_config_noss =
{
    .sck_pin        = DAC_SPI_CK_PIN,
    .mosi_pin       = DAC_SPI_SI_PIN,
    .miso_pin       = NRF_DRV_SPI_PIN_NOT_USED,
    .ss_pin         = NRF_DRV_SPI_PIN_NOT_USED,
    .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
    .orc            = 0xFF,
    .frequency      = NRF_DRV_SPI_FREQ_8M,
    .mode           = NRF_DRV_SPI_MODE_1,
    .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
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
#ifdef DAC_USE_SPI_MNGR
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
#endif

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
#ifdef DAC_SPI_USE_MNGR
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
#endif

#ifdef DAC_USE_SPI_MNGR
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
#endif

static void dac_update_recycle()
{
    
}

#ifdef DAC_USE_SPI_MNGR
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
#endif

#ifdef DAC_USE_SPI_MNGR
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
#endif

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
 *  We expect both prints work, indicating that ppi and interrupt handler could work simultaneously.
 */
static void test1_run(void)
{
    (void)test1_run;
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
 * In test2, the lifespan of cycle timer contains five cycle or burst timer.
 *
 * burst timer is one-shot, log a number in interrupt.
 * cycle timer compares to 1s, ppi to increment counter and to start burst timer as well.
 * counter set to compare 5.
 * 
 */
static void test2_run(void)
{
    (void)test2_run;
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

#ifdef DAC_USE_SPI_MNGR
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
#endif

#ifndef DAC_USE_SPI_MNGR
/**
 * test3 toggle all 
 */
static void test3_run(void)
{
    (void)test3_run;
    
    NRF_LOG_INFO("test3, broadcast 1/0 in a loop, 1 sec interval");
    
    static uint8_t pu[2] = { 0xD0, 0x00 };  // clear powerdown on all channels
    static uint8_t hi[2] = { 0xCF, 0xF0 };  // broadcast all to high
    static uint8_t lo[2] = { 0xC0, 0x00 };  // broadcast all to low
    
    // no callback, blocking mode
    nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    
    // power up all channels
    nrf_drv_spi_transfer(&m_dac_spi, pu, 2, NULL, 0);
    
    for (;;)
    {
        nrf_drv_spi_transfer(&m_dac_spi, hi, 2, NULL, 0);
        vTaskDelay(512);
        nrf_drv_spi_transfer(&m_dac_spi, lo, 2, NULL, 0);
        vTaskDelay(512);
    }
    
    vTaskDelay(portMAX_DELAY);
}

static void test4_run(void)
{
    (void)test4_run;
    
    NRF_LOG_INFO("test04, power down all but H (VOUTD,PORTD), toggle VOUTD (TP7)");
    
    static uint8_t pd[2] = { 0xD0, 0x3F };  // power down all but h
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t hh[2] = { 0x78, 0x00 };  // set channel h high
    static uint8_t hl[2] = { 0x70, 0x00 };  // set channel h low
    
    nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    nrf_drv_spi_transfer(&m_dac_spi, pd, 2, NULL, 0);
    nrf_drv_spi_transfer(&m_dac_spi, wt, 2, NULL, 0);
    
    for (;;)
    {
        nrf_drv_spi_transfer(&m_dac_spi, hh, 2, NULL, 0);
        vTaskDelay(512);
        nrf_drv_spi_transfer(&m_dac_spi, hl, 2, NULL, 0);
        vTaskDelay(512);        
    }
}

static void test5_run(void)
{
    (void)test5_run;
    
    NRF_LOG_INFO("test 5, power down all but G (VOUTC,PORTC) and toggle");
    NRF_LOG_INFO("  VOUTC (TP9) should toggle");
    
    static uint8_t pd[2] = { 0xD0, 0xBF };  // power down all but g
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t gh[2] = { 0x68, 0x00 };  // set channel g high
    static uint8_t gl[2] = { 0x60, 0x00 };  // set channel g low
    
    nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    nrf_drv_spi_transfer(&m_dac_spi, pd, 2, NULL, 0);
    nrf_drv_spi_transfer(&m_dac_spi, wt, 2, NULL, 0);
    
    for (;;)
    {
        nrf_drv_spi_transfer(&m_dac_spi, gh, 2, NULL, 0);
        vTaskDelay(512);
        nrf_drv_spi_transfer(&m_dac_spi, gl, 2, NULL, 0);
        vTaskDelay(512);        
    }
}

static void test6_run(void)
{
    (void)test6_run;
    
    NRF_LOG_INFO("test 6, short VOUTD (H) and VMID (2.5V), toggle G (VOUTC - TP9, PORTC)");
    
    static uint8_t pd[2] = { 0xD0, 0x3F };  // power down all but g and h
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t hm[2] = { 0x78, 0x00 };  // set channel h to half
    static uint8_t gh[2] = { 0x6E, 0xB0 };  // set channel g high
    static uint8_t gl[2] = { 0x61, 0x00 };  // set channel g low
    
    nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, NULL, NULL);
    nrf_drv_spi_transfer(&m_dac_spi, pd, 2, NULL, 0);
    nrf_drv_spi_transfer(&m_dac_spi, wt, 2, NULL, 0);
    nrf_drv_spi_transfer(&m_dac_spi, hm, 2, NULL, 0);    
    
    for (;;)
    {
        nrf_drv_spi_transfer(&m_dac_spi, gh, 2, NULL, 0);    
        vTaskDelay(512);
        nrf_drv_spi_transfer(&m_dac_spi, gl, 2, NULL, 0);    
        vTaskDelay(512);        
    }
}

static void test7_burst_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("test7 burst timer cb");
}

static void test7_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{       
    NRF_LOG_INFO("test 7 spi evt handler");
}

/**
 * test7 tries to repeatedly start spi (hold) xfer by ppi. Also, ss signal triggered by the SAME event source.
 * 
 * burst timer compare0 is used as event source, triggering ss toggle (cs low), as well as spi task using fork.
 * spi end triggers the other ss toggle (cs high) in the same cycle.
 * burst timer also has a rewind (compare1) for repetition.
 * 
 * a dummy write (wtm) used as spi xfer data. no verified. ss signal and spi clock are verified as expected on 
 * oscilloscope.
 */
static void test7_run(void)
{
    (void)test7_run;
    
    uint32_t err, event_addr, task_addr;
    
    // gpio init ss pin
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);
    
    // spi init
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test7_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);
    
    // timer init
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test7_burst_timer_callback);
    APP_ERROR_CHECK(err);    
    
    // trigger
    nrf_drv_timer_compare(&m_burst_timer, 
                          NRF_TIMER_CC_CHANNEL0, 
                          1000 * 1000, // 100, 
                          true);
    // rewind, 100Hz
    nrf_drv_timer_extended_compare(&m_burst_timer,
                                   NRF_TIMER_CC_CHANNEL1,
                                   3000 * 1000, // 10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
                                   false);
    // ppis
    nrf_ppi_channel_t ppic_timc0;
    nrf_ppi_channel_t ppic_spi_end;
    
    // timer trigger ss
    err = nrfx_ppi_channel_alloc(&ppic_timc0);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppic_timc0, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    err = nrfx_ppi_channel_fork_assign(ppic_timc0, task_addr);
    
    err = nrfx_ppi_channel_enable(ppic_timc0);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_alloc(&ppic_spi_end);
    APP_ERROR_CHECK(err);

    event_addr = nrf_drv_spi_end_event_get(&m_dac_spi);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppic_spi_end, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppic_spi_end);
    APP_ERROR_CHECK(err);
    
    // spi start
    static uint8_t wt[2] = { 0x90, 0x00 }; // set wtm mode
    static nrf_drv_spi_xfer_desc_t xfer = { 
        .p_tx_buffer = wt,
        .tx_length = 2
    };
    
    uint32_t flags = 
        NRF_DRV_SPI_FLAG_HOLD_XFER |
        // NRF_DRV_SPI_FLAG_RX_POSTINC |
        // NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER |
        NRF_DRV_SPI_FLAG_REPEATED_XFER;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, flags);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_burst_timer);
}

static void test8_burst_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    // NRF_LOG_INFO("test8 burst timer cb");
}

static void test8_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("test8 count timer cb");
}

static void test8_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{       
    // NRF_LOG_INFO("test8 spi evt handler");
}


/**
 * test7 tries to repeatedly start spi (hold) xfer by ppi. Also, ss signal triggered by the SAME event source.
 * 
 * burst timer compare0 is used as event source, triggering ss toggle (cs low), as well as spi task using fork.
 * spi end triggers the other ss toggle (cs high) in the same cycle.
 * burst timer also has a rewind (compare1) for repetition.
 * 
 * a dummy write (wtm) used as spi xfer data. no verified. ss signal and spi clock are verified as expected on 
 * oscilloscope.

 * test8 adds a count timer, counts up to N (=8), insisting on cs low in the beginning and cs high in the end (
 * aka. cs normally high).
 *
 * which signal should be used for cs high in each cycle? test8a uses spi end event. This leaves a wide gap
 * between spi clk/data end and spi high. the minimal cycle is 49. for 8 words this translates into 24.5uS. Or
 * for better safety, use 50 (25uS)
 * 
 * burst timer c0 starts spi. c1 for cs low, as well as counting.
 * count timer c0 is oneshot and stops burst timer.
 * count timer is started manually. no trigger.
 * spi-end triggers all cs high.
 * 
 * if this two timers work together as an inside loop. there should be a (repeated) trigger in outside loop to 
 * start both. supposedly. This should be verified in next test.
 
 * alternatively. use burst timer c2 (rewind) to trigger cs high and to count, in this case c0 must be at least 4.
 */
static void test8a_run(void)
{
    (void)test7_run;
    
    uint32_t err, event_addr, task_addr;
    
    // gpio init ss pin
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);
    
    // spi reinit
    static dac_spi_ctx_t ctx = { 0 };
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test8_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);
    
    // burst timer init
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_16MHz;
    err = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test8_burst_timer_callback); // TODO remove cb?
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

    err = nrf_drv_timer_init(&m_count_timer, &count_cfg, test8_count_timer_callback);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_extended_compare(&m_count_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   2,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                   true);
    
    // ppis
    nrf_ppi_channel_t ppic_timc0;   // starts spi and count
    nrf_ppi_channel_t ppic_timc1;   // assert ss
    // nrf_ppi_channel_t ppic_timc2;   // replace ppic_spi_end
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
    
//    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
//    err = nrfx_ppi_channel_fork_assign(ppic_spi_end, task_addr);
//    APP_ERROR_CHECK(err);    
    
    err = nrfx_ppi_channel_enable(ppic_spi_end);
    APP_ERROR_CHECK(err);
    
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
    nrf_drv_timer_enable(&m_burst_timer);
}

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

static void test9a_count_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    // test9a_one_cycle_start();
    NRF_LOG_INFO("count timer finished");
}

static void test9a_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("cycle timer fired");
    // test9a_one_cycle_start();
    
    // NRF_LOG_INFO("count timer enabled %d", nrf_drv_timer_is_enabled(&m_count_timer) ? 1 : 0);
    // NRF_LOG_INFO("burst timer enabled %d", nrf_drv_timer_is_enabled(&m_burst_timer) ? 1 : 0);
    
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
static void test9a_run(void)
{
    (void)test7_run;
    
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
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, test8_spi_event_handler, &ctx);
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
    err = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test8_burst_timer_callback); // TODO remove cb?
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
    // nrf_ppi_channel_t ppic_timc2;   // replace ppic_spi_end
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
    
//    task_addr = nrfx_timer_task_address_get(&m_count_timer, NRF_TIMER_TASK_COUNT);
//    err = nrfx_ppi_channel_fork_assign(ppic_spi_end, task_addr);
//    APP_ERROR_CHECK(err);    
    
    err = nrfx_ppi_channel_enable(ppic_spi_end);
    APP_ERROR_CHECK(err);
    
    // spi start
//    static uint8_t data[16] = { 
//        0x60, 0x00, 
//        0x64, 0x00, 
//        0x68, 0x00, 
//        0x6c, 0x00,
//        0x6f, 0xf0, 
//        0x6c, 0x00, 
//        0x68, 0x00, 
//        0x64, 0x00,        
//    };
//    static nrf_drv_spi_xfer_desc_t xfer = { 
//        .p_tx_buffer = data,
//        .tx_length = 2
//    };
//    
//    uint32_t flags = 
//        NRF_DRV_SPI_FLAG_HOLD_XFER |
//        NRF_DRV_SPI_FLAG_TX_POSTINC |
//        NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER |
//        NRF_DRV_SPI_FLAG_REPEATED_XFER;
//    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, flags);
//    APP_ERROR_CHECK(err);

    test9a_spi_xfer();
    
    nrf_drv_timer_enable(&m_count_timer);
    nrf_drv_timer_enable(&m_burst_timer);
    nrf_drv_timer_enable(&m_cycle_timer);
    
//      test9a_one_cycle_start();

}
#endif

// 100Hz (10 * 1000us) period, 50uS pulse width 
// single timer
// init delay 10us
// on event
// off event
static void test20_cycle_timer_callback(nrf_timer_event_t event_type, void * p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("test20 c0 fired");

            break;
        case NRF_TIMER_EVENT_COMPARE1:
            NRF_LOG_INFO("test20 c1 fired");

            break;
        case NRF_TIMER_EVENT_COMPARE2:
            NRF_LOG_INFO("test20 c2 fired");
            break;
        default:
            break;
    }    
}    

static void test20_timer_init(void)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test20_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
}

#ifndef DAC_USE_SPI_MNGR
static void dummy_spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{   
    ret_code_t err;
    static uint8_t gh[2] = { 0x6F, 0xF0 };  // set channel g high
    static uint8_t gl[2] = { 0x60, 0x00 };  // set channel g low
    static nrf_drv_spi_xfer_desc_t xfer_gh = { .p_tx_buffer = gh, .tx_length = 2, };
    static nrf_drv_spi_xfer_desc_t xfer_gl = { .p_tx_buffer = gl, .tx_length = 2, };
    
    dac_spi_ctx_t *p_ctx = (dac_spi_ctx_t *)p_context;
    
    if (p_ctx->bypass) {
        NRF_LOG_INFO("spi event bypass");
        return;
    }
    
    if (p_ctx->value == 0) 
    {
        err = nrf_drv_spi_xfer(&m_dac_spi, &xfer_gl, NRF_DRV_SPI_FLAG_HOLD_XFER);
        APP_ERROR_CHECK(err);        
    } 
    else 
    {
        err = nrf_drv_spi_xfer(&m_dac_spi, &xfer_gh, NRF_DRV_SPI_FLAG_HOLD_XFER);
        APP_ERROR_CHECK(err);       
    }
    p_ctx->value = !p_ctx->value;
}

/**
 * burst timer used in this test, with rise, fall, and rewind compares. Rewind is set to 100Hz.
 * rise triggers spi task to rise g
 * fall triggers spi task to fall g
 * for example:
 * 1 init
 * 2 xfer (hold) high
 * 3 loop timer
 *   rise ppi> spi task -> done interrupt -> code toggle xfer (hold)
 *   fall ppi> spi task -> done interrupt -> code toggle xfer (hold)
 *
 * !!! PROBLEM !!!
 * it is seen that the pulse may be inverted due interrupt overlap or missing.
 * on oscilloscope we can see the spi interface signal, cs and clock (or data), the cs pull down 
 * persists almost 60us, which is too high. we try to solve this problem in next test, using PPI to control 
 * cs pin, rather than spi driver.
 */
static void test20_run(void)
{
    (void)test20_run;
    
    nrfx_err_t err;
    nrf_ppi_channel_t ppi_channel_rise;
    nrf_ppi_channel_t ppi_channel_fall;
    
    static dac_spi_ctx_t ctx = { 0 };
    
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config, dummy_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);
    
    test20_timer_init();

    // rise
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL0, 
                          100,
                          false);
    // fall
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          160,
                          false);
    
    // rewind
    nrf_drv_timer_extended_compare(&m_burst_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);
    // rise ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise);
    APP_ERROR_CHECK(err);
    
    uint32_t rise_event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t rise_task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi_channel_rise, rise_event_addr, rise_task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_rise);
    APP_ERROR_CHECK(err);

    // fall ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall);
    APP_ERROR_CHECK(err);
    
    uint32_t fall_event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL1);
    uint32_t fall_task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi_channel_fall, fall_event_addr, fall_task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_fall);
    APP_ERROR_CHECK(err);
    
    /* init dac */
    static uint8_t pd[2] = { 0xD0, 0x3F };  // power down all but g and h
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t hm[2] = { 0x78, 0x00 };  // set channel h to half
    static uint8_t gh[2] = { 0x6E, 0xB0 };  // set channel g high
    static uint8_t gl[2] = { 0x61, 0x00 };  // set channel g low
    
    static nrf_drv_spi_xfer_desc_t xfer = { .tx_length = 2, };
    xfer.p_tx_buffer = pd;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(10);
    
    xfer.p_tx_buffer = wt;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(10);    
    
    xfer.p_tx_buffer = hm;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(10);
    
    // assure low is seen on oscilloscope for 3 seconds
    xfer.p_tx_buffer = gl;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(3000);
    
    
    xfer.p_tx_buffer = gh;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_HOLD_XFER);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_burst_timer);
    NRF_LOG_INFO("test20: burst timer started");
}

static void test21_run(void)
{
    (void)test21_run;
    
    uint32_t err, event_addr, task_addr;
    
    err = nrfx_gpiote_init();
    APP_ERROR_CHECK(err);
    
    // nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    nrfx_gpiote_out_config_t ss_pin_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // task pin, initial high
    err = nrfx_gpiote_out_init(DAC_SPI_SS_PIN, &ss_pin_config);
    APP_ERROR_CHECK(err);
    
    nrfx_gpiote_out_task_enable(DAC_SPI_SS_PIN);
    
    nrf_ppi_channel_t ppi_channel_rise;
    nrf_ppi_channel_t ppi_channel_rise_ss;

    nrf_ppi_channel_t ppi_channel_fall;
    nrf_ppi_channel_t ppi_channel_fall_ss;

    nrf_ppi_channel_t ppi_channel_spi_end;
    
    static dac_spi_ctx_t ctx = { 
        .bypass = true,
    };
    
    err = nrf_drv_spi_init(&m_dac_spi, &m_dac_spi_config_noss, dummy_spi_event_handler, &ctx);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, test20_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err);

    // rise
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL0, 
                          100,
                          false);
    // fall
    nrf_drv_timer_compare(&m_burst_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          160,
                          false);
    
    // rewind
    nrf_drv_timer_extended_compare(&m_burst_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);
    // rise ppi ss
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise_ss);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppi_channel_rise_ss, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_rise_ss);
    APP_ERROR_CHECK(err);
    
    // rise ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi_channel_rise, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_rise);
    APP_ERROR_CHECK(err);
    
    // fall ppi ss
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall_ss);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL1);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppi_channel_fall_ss, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_fall_ss);
    APP_ERROR_CHECK(err);    
    
    // fall ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall);
    APP_ERROR_CHECK(err);
    
    event_addr = nrfx_timer_compare_event_address_get(&m_burst_timer, NRF_TIMER_CC_CHANNEL1);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);
    
    err = nrfx_ppi_channel_assign(ppi_channel_fall, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_fall);
    APP_ERROR_CHECK(err);
    
    // spi end
    err = nrfx_ppi_channel_alloc(&ppi_channel_spi_end);
    APP_ERROR_CHECK(err);
    
    event_addr = nrf_drv_spi_end_event_get(&m_dac_spi);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);
    // task_addr = nrfx_gpiote_set_task_addr_get(DAC_SPI_SS_PIN);
    
    err = nrfx_ppi_channel_assign(ppi_channel_spi_end, event_addr, task_addr);
    APP_ERROR_CHECK(err);
    
    err = nrfx_ppi_channel_enable(ppi_channel_spi_end);
    APP_ERROR_CHECK(err);
    
    
    /* init dac */
    static uint8_t pd[2] = { 0xD0, 0x3F };  // power down all but g and h
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t hm[2] = { 0x78, 0x00 };  // set channel h to half
    static uint8_t gh[2] = { 0x6E, 0xB0 };  // set channel g high
    static uint8_t gl[2] = { 0x61, 0x00 };  // set channel g low
    
    static nrf_drv_spi_xfer_desc_t xfer = { .tx_length = 2, };
    
    // https://infocenter.nordicsemi.com/topic/sdk_nrf5_v17.1.0/hardware_driver_spi_master.html
    // !!! NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER suppress end event
    
    for (;;) 
    {
        xfer.p_tx_buffer = pd;
        nrfx_gpiote_out_task_trigger(DAC_SPI_SS_PIN);
        err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, 0);
        APP_ERROR_CHECK(err);
        vTaskDelay(1);
        // nrfx_gpiote_out_set(DAC_SPI_SS_PIN);
        
        vTaskDelay(1000);
    }
    
    xfer.p_tx_buffer = wt;
    nrfx_gpiote_out_clear(DAC_SPI_SS_PIN);
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(1);
    nrfx_gpiote_out_set(DAC_SPI_SS_PIN);
    
    xfer.p_tx_buffer = hm;
    nrfx_gpiote_out_clear(DAC_SPI_SS_PIN);
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(1);
    nrfx_gpiote_out_set(DAC_SPI_SS_PIN);
    
    // assure low is seen on oscilloscope for 3 seconds
    xfer.p_tx_buffer = gl;
    nrfx_gpiote_out_clear(DAC_SPI_SS_PIN);
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER);
    APP_ERROR_CHECK(err);
    vTaskDelay(1);
    nrfx_gpiote_out_set(DAC_SPI_SS_PIN);
    
    vTaskDelay(3000);
    
    xfer.p_tx_buffer = gh;
    err = nrf_drv_spi_xfer(&m_dac_spi, &xfer, NRF_DRV_SPI_FLAG_HOLD_XFER);
    APP_ERROR_CHECK(err);
    
    nrf_drv_timer_enable(&m_burst_timer);
    NRF_LOG_INFO("test20: burst timer started");
}
#endif

#ifdef DAC_USE_SPI_MNGR
static void timer_init(void)
{
    uint32_t err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&m_cycle_timer, &timer_cfg, cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_timer_init(&m_burst_timer, &timer_cfg, burst_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
}
#endif

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

// static void stim_prepare(void)
// {
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
    
//    NRF_LOG_INFO("howland stim prepared");
// }

//static void stim_start(void)
//{
////    asw_on();
////    nrf_drv_timer_enable(&m_cycle_timer);
//    
//    NRF_LOG_INFO("howland stim started");
//}

//static void stim_stop(void)
//{
////    nrf_drv_timer_disable(&m_cycle_timer);
////    asw_off();
//    
//    NRF_LOG_INFO("howland stim stopped");
//}

/**
 * Howland
 */
#ifdef DAC_USE_SPI_MNGR
static void howland_task(void * pvParameters)
{
    // test3_run();
    // test6_run();
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
                    // stim_stop();
                    m_msg.c = 0;
                    stim_started = false;
                }
                ble_nus_send((uint8_t *)&m_msg, &msg_length);
                break;
            case 2: { // START // TODO check p_msg and reject
                if (stim_started)
                {
                    // stim_stop();    
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
#else
static void howland_task(void * pvParameters)
{
    // test21_run();
    test9a_run();
    vTaskDelay(portMAX_DELAY);
}
#endif


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
