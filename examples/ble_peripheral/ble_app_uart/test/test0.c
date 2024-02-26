#include "nrf_drv_spi.h"

#include "nrf_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include "test.h"

extern nrf_drv_spi_t const m_dac_spi;
extern nrf_drv_spi_config_t const m_dac_spi_config;

void test0a_run(void)
{
    NRF_LOG_INFO("test0a, broadcast 1/0 in a loop, 1 sec interval");
    
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
        vTaskDelay(10);
        nrf_drv_spi_transfer(&m_dac_spi, lo, 2, NULL, 0);
        vTaskDelay(10);
    }
}

void test0b(void)
{
}

