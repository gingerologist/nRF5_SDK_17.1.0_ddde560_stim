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

void test4a(void)
{
    NRF_LOG_INFO("test4 powers down all but channel h and toggle");

    static uint8_t pd[2] = { 0xD0, 0x3F };  // power down all but h
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t hh[2] = { 0x7f, 0xf0 };  // set channel h high
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

void test4b(void)
{
    NRF_LOG_INFO("test5 powers down all channel but g and toggle");

    static uint8_t pd[2] = { 0xD0, 0xBF };  // power down all but g
    static uint8_t wt[2] = { 0x90, 0x00 };  // set wtm mode
    static uint8_t gh[2] = { 0x6f, 0xf0 };  // set channel g high
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

/* connect h and vmid to fix the floating vmid problem (no buffered) */
void test4c(void)
{
    NRF_LOG_INFO("test4c, short h to VMID and toggle g");

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
