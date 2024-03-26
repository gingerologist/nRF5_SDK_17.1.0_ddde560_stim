#ifndef __HOWLAND_H__
#define __HOWLAND_H__

#include <stdint.h>
#include <stdbool.h>
#include "app_util.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"

#define NOT_INSIDE_ISR          (( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk ) == 0 )
#define INSIDE_ISR              (!(NOT_INSIDE_ISR))

/**
 * Burst timer generates rising and falling edge ppi, as well as interrupt (for reconfigure)
 */
#define COUNT_TIMER_ID                      1       // counter
#define BURST_TIMER_ID                      2
#define CYCLE_TIMER_ID                      3

#define DAC_SPI_INSTANCE                    1

// #define DAC_SPI_SS_PIN                      14
// #define DAC_SPI_CK_PIN                      12
// #define DAC_SPI_SI_PIN                      1

#define DAC_SPI_SS_PIN                      1
#define DAC_SPI_CK_PIN                      0
#define DAC_SPI_SI_PIN                      6

#define IDT_TWI_INSTANCE                    0
#define IDT_TWI_MAX_PENDING_TRANSACTIONS    5

#define IDT_I2C_ADDR                        0x61
#define IDT_SCL_PIN                         21
#define IDT_SDA_PIN                         18 // 24 error in schematic net name
#define IDT_INT_PIN                         16
#define IDT_PDETB_PIN                       15

#define IDT_REG_CHIP_ID_L                   0x00
#define IDT_REG_CHIP_ID_H                   0x01
#define IDT_REG_CHIP_REV                    0x02

#define IDT_REG_STATUS_L                    0x34
#define IDT_REG_STATUS_H                    0x35

#define IDT_REG_INT_L                       0x36
#define IDT_REG_INT_H                       0x37

#define IDT_REG_INT_ENABLE_L                0x38
#define IDT_REG_INT_ENABLE_H                0x39

#define IDT_REG_INT_CLEAR_L                 0x3A
#define IDT_REG_INT_CLEAR_H                 0x3B

#define IDT_REG_VOUT_SET                    0x3C
#define IDT_REG_ILIM_SET                    0x3D
#define IDT_REG_CHG_STATUS                  0x3E
#define IDT_REG_EPT                         0x3F

#define IDT_REG_ADC_VRECT_L                 0x40
#define IDT_REG_ADC_VRECT_H                 0x41

#define IDT_REG_ADC_VOUT_L                  0x42
#define IDT_REG_ADC_VOUT_H                  0x43

#define IDT_REG_IOUT_L
#define IDT_REG_IOUT_H

#define IDT_REG_OP_FREQ_L
#define IDT_REG_OP_FREQ_H

#define IDT_REG_SYS_OP_MODE                 0x4C

#define IDT_REG_COMMAND                     0x4E



#define IDT_REG_VRECT_MIN_CORRECT           0x96 // 16bit


/**
 * 1. timed mode stimulation, timeout non-zero, countdown non-zero, less than 0xf000
 *                            timeout non-zero, countdown zero
 * 2. infinite mode stim,     timeout 0, no need to have this mode, timeout could be a max value
 * 3. for command
 * if num_of_pulses is 0 then all timing ignored.
 */

typedef struct /* __attribute__((packed)) */ ble_incoming_message 
{                               // example
    uint16_t c;                 // 02 00        command GET = 0, STOP = 1, START = 2
                                //              countdown for values below 0x00 0xFF
    uint16_t timeout;           // 00 00        not used yet
    
    uint16_t freq;              // 00 00        
    uint16_t num_of_pulses;     // 01 00        not used, defaults to 1
    uint16_t pulse_width;       // 32 00
    uint16_t pulse_interval;    // 00 00        not used yet
    uint16_t hold;              // 00 00        not used yet
    uint16_t recycle_ratio;     // C8 00

    int8_t current[8];          // 00 00 0A F6 0 0 0 0
} ble_incomming_message_t;

STATIC_ASSERT(sizeof(ble_incomming_message_t) == 24);

// nrf_drv_spi context
typedef struct dac_spi_ctx {
    int value;
    bool bypass;
} dac_spi_ctx_t;

void howland_freertos_init(void);

/**
 * This is a wrapper for ble_nus_data_send
 * main.c should implement this function and 
 * howland uses it to reply
 */
void ble_nus_send(uint8_t * p_data, uint16_t * p_length);

/**
 * These functions are wrappers for message dequeue/enqueue,
 * implemented in howland.c
 * used in main (TODO what is the context?)
 */ 
ble_incomming_message_t * ble_nus_alloc(void);
ble_incomming_message_t * ble_nus_alloc_from_isr(void);
void ble_nus_recv(ble_incomming_message_t * p_message);
void ble_nus_recv_from_isr(ble_incomming_message_t * p_message);

extern nrf_drv_spi_t const m_dac_spi;
extern nrf_drv_spi_config_t const m_dac_spi_config;

extern nrf_drv_spi_config_t const m_dac_spi_config;
extern nrf_drv_spi_config_t const m_dac_spi_config_noss;

extern nrf_drv_timer_t const m_count_timer;
extern nrf_drv_timer_t const m_cycle_timer;
extern nrf_drv_timer_t const m_spi_timer;



#endif
