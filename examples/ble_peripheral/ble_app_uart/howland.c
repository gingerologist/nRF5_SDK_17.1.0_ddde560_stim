#include "nrf_spi_mngr.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "howland.h"

NRF_SPI_MNGR_DEF(m_spi_mngr, 16, 0);

static TaskHandle_t m_thread = NULL;

static nrf_drv_spi_config_t const m_dac_config =
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
    
    nrf_spi_mngr_perform(&m_spi_mngr, &m_dac_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
    
    _mode = mode;
}

// see 
static const uint16_t dac_wrm_mode = 0x8000;
static const uint16_t dac_wtm_mode = 0x9000;
static const uint16_t dac_cmd1 = 0xA000;
static const uint16_t dac_cmd2 = 0xB000;

static nrf_spi_mngr_transfer_t const dac_wrm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wrm_mode, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t const dac_wtm_mode_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_wtm_mode, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t const dac_cmd1_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd1, 2, NULL, 0) };
static nrf_spi_mngr_transfer_t const dac_cmd2_xfer[] = { NRF_SPI_MNGR_TRANSFER(&dac_cmd1, 2, NULL, 0) };

static uint16_t dac_reg[8] = { 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800, 0x0800 };
static uint16_t *dac_cha = &dac_reg[0];
static uint16_t *dac_chb = &dac_reg[1];
static uint16_t *dac_chc = &dac_reg[2];
static uint16_t *dac_chd = &dac_reg[3];
static uint16_t *dac_che = &dac_reg[4];
static uint16_t *dac_chf = &dac_reg[5];
static uint16_t *dac_chg = &dac_reg[6];
static uint16_t *dac_chh = &dac_reg[7];

// this function update 8 channels simultaneously
// there are two ways to do so
// 1. set wrm mode, update 8 channels, special command 1
// 2. set wrm mode, update 7 channles, special command 2
// here we use the second method.
static void dac_update_simult(uint16_t data[8])
{
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
    
    nrf_spi_mngr_perform(&m_spi_mngr, &m_dac_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
}

static void dac_update_sequent(uint16_t data[8])
{
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
    
    nrf_spi_mngr_perform(&m_spi_mngr, &m_dac_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);    
}

static void dac_update_same(uint8_t val)
{
    uint16_t data = (((uint16_t)val) << 4) | 0xC000;
    
    nrf_spi_mngr_transfer_t xfers[] =
    {
        NRF_SPI_MNGR_TRANSFER(&data, 2, NULL, 0),
    };
    
    nrf_spi_mngr_perform(&m_spi_mngr, &m_dac_config, xfers, sizeof(xfers) / sizeof(xfers[0]), NULL);
}

static void howland_task(void * pvParameters)
{
    ret_code_t ret = nrf_spi_mngr_init(&m_spi_mngr, &m_dac_config);
    
    
                         
    vTaskDelay(portMAX_DELAY);                         
    
    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
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
