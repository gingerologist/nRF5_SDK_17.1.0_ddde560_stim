#include "nrf_drv_spi.h"
#include "nrf_spi_mngr.h"
#include "nrf_drv_timer.h"
// #include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"
// #include "nrfx_spim.h"

#include "nrf_log.h"
#include "nrf_twi_mngr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "howland.h"
#include "test\test.h"

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, IDT_TWI_MAX_PENDING_TRANSACTIONS, IDT_TWI_INSTANCE);

static uint8_t idt_txbuf[16] = { 0 };
static uint8_t idt_rxbuf[16] = { 0 };

// TWI (with transaction manager) initialization.
static void twi_config(void) {
  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
      .scl = IDT_SCL_PIN,
      .sda = IDT_SDA_PIN,
      .frequency = NRF_DRV_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
      .clear_bus_init = false
  };

  err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
  APP_ERROR_CHECK(err_code);
}

static void read_reg(uint8_t addr, uint8_t numOfRegs) 
{
    uint32_t err;
    static nrf_twi_mngr_transfer_t rxfers[] = {
        NRF_TWI_MNGR_WRITE(IDT_I2C_ADDR, idt_txbuf, 2, NRF_TWI_MNGR_NO_STOP),
        NRF_TWI_MNGR_READ (IDT_I2C_ADDR, idt_rxbuf, 1, 0)
    };    
    
    idt_txbuf[0] = 0x00;
    idt_txbuf[1] = 0x3C;
    idt_txbuf[2] = 0x78;
    
    err = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, rxfers, 2, NULL);
    APP_ERROR_CHECK(err);
    
    NRF_LOG_INFO("read 0x3c reg %x", idt_rxbuf[0]);
    
    static nrf_twi_mngr_transfer_t txfers[] = {
        NRF_TWI_MNGR_WRITE(IDT_I2C_ADDR, idt_txbuf, 3, NULL),
    };  

    err = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, txfers, 3, 0);
    APP_ERROR_CHECK(err);    
    
    err = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, rxfers, 2, NULL);
    APP_ERROR_CHECK(err);
    
    NRF_LOG_INFO("read 0x3c reg %x", idt_rxbuf[0]);    
}

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
 */

/* This has been fixed in v2 board. 
 *
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
 */

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
nrf_drv_spi_t const m_dac_spi = NRF_DRV_SPI_INSTANCE(DAC_SPI_INSTANCE);  /**< SPI instance. */

nrf_drv_spi_config_t const m_dac_spi_config =
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

/*
 * noss (3-wire) version is used. ss pin is controlled by timer/ppi
 */
nrf_drv_spi_config_t const m_dac_spi_config_noss =
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

// see
// static uint16_t dac_wrm_mode = 0x8000;
// static uint16_t dac_wtm_mode = 0x9000;
// static uint16_t dac_cmd_update_select = 0xA000;
// static uint16_t dac_cmd_update_all = 0xA0FF;
// static uint16_t dac_cmd_chan_a_write = 0xB000;
// static uint16_t dac_cmd_broadcast = 0xC000;

// static nrf_spi_mngr_transfer_t dac_wrm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wrm_mode, 2, NULL, 0) };
// static nrf_spi_mngr_transfer_t dac_wtm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wtm_mode, 2, NULL, 0) };
// static nrf_spi_mngr_transfer_t dac_cmd_update_select_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_update_select, 2, NULL, 0) };
// static nrf_spi_mngr_transfer_t dac_cmd_chan_a_write_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_chan_a_write, 2, NULL, 0) };
// static nrf_spi_mngr_transfer_t dac_cmd_broadcast_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd_broadcast, 2, NULL, 0) };

// static uint16_t dac_reg_init[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };
// static uint16_t dac_reg_pulse[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };
// static uint16_t dac_reg_recycle[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };

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

// in v2 board, analog switch removed.

/**
 * Timer
 */
const nrf_drv_timer_t m_seg_counter = NRF_DRV_TIMER_INSTANCE(SEG_COUNTER_ID);
const nrf_drv_timer_t m_cyc_counter = NRF_DRV_TIMER_INSTANCE(CYC_COUNTER_ID);

const nrf_drv_timer_t m_spi_timer = NRF_DRV_TIMER_INSTANCE(SPI_TIMER_ID);
const nrf_drv_timer_t m_seg_timer = NRF_DRV_TIMER_INSTANCE(SEG_TIMER_ID);

#if 0 // TODO

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
    err_code = nrf_drv_timer_init(&m_spi_timer, &timer_cfg, test20_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err_code);
}

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
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL0,
                          100,
                          false);
    // fall
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          160,
                          false);

    // rewind
    nrf_drv_timer_extended_compare(&m_spi_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);
    // rise ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise);
    APP_ERROR_CHECK(err);

    uint32_t rise_event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t rise_task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);

    err = nrfx_ppi_channel_assign(ppi_channel_rise, rise_event_addr, rise_task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi_channel_rise);
    APP_ERROR_CHECK(err);

    // fall ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall);
    APP_ERROR_CHECK(err);

    uint32_t fall_event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL1);
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

    nrf_drv_timer_enable(&m_spi_timer);
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
    err = nrf_drv_timer_init(&m_spi_timer, &timer_cfg, test20_cycle_timer_callback);  // TODO
    APP_ERROR_CHECK(err);

    // rise
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL0,
                          100,
                          false);
    // fall
    nrf_drv_timer_compare(&m_spi_timer,
                          NRF_TIMER_CC_CHANNEL1,
                          160,
                          false);

    // rewind
    nrf_drv_timer_extended_compare(&m_spi_timer,
                                   NRF_TIMER_CC_CHANNEL2,
                                   10 * 1000,
                                   NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                   false);
    // rise ppi ss
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise_ss);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppi_channel_rise_ss, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi_channel_rise_ss);
    APP_ERROR_CHECK(err);

    // rise ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_rise);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL0);
    task_addr = nrf_drv_spi_start_task_get(&m_dac_spi);

    err = nrfx_ppi_channel_assign(ppi_channel_rise, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi_channel_rise);
    APP_ERROR_CHECK(err);

    // fall ppi ss
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall_ss);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL1);
    task_addr = nrfx_gpiote_out_task_addr_get(DAC_SPI_SS_PIN);

    err = nrfx_ppi_channel_assign(ppi_channel_fall_ss, event_addr, task_addr);
    APP_ERROR_CHECK(err);

    err = nrfx_ppi_channel_enable(ppi_channel_fall_ss);
    APP_ERROR_CHECK(err);

    // fall ppi
    err = nrfx_ppi_channel_alloc(&ppi_channel_fall);
    APP_ERROR_CHECK(err);

    event_addr = nrfx_timer_compare_event_address_get(&m_spi_timer, NRF_TIMER_CC_CHANNEL1);
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

    nrf_drv_timer_enable(&m_spi_timer);
    NRF_LOG_INFO("test20: burst timer started");
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
#endif

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

#endif


/******************************************************************************
 *
 *
 */


/*
 * configurations;
 * 1. spi timer, channel 0
 *
 */
static void stim_start(void)
{
    
}


static void howland_task(void * pvParameters)
{   
    test6e();
    vTaskDelay(portMAX_DELAY);
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
