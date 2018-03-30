//180315 SPI-S

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_spi.h"

#define SPI0_INSTANCE  0 /**< SPI0 instance index. */
#define SPI1_INSTANCE  1 /**< SPI1 instance index. */
static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI0_INSTANCE);  /**< SPI instance. */
static const nrf_drv_spi_t spi1 = NRF_DRV_SPI_INSTANCE(SPI1_INSTANCE);  /**< SPI instance. */
static volatile bool spi0_xfer_done;  /**< ADC Flag used to indicate that SPI instance completed the transfer. */
static volatile bool spi1_xfer_done;  /**< DAC-CS0 Flag used to indicate that SPI instance completed the transfer. */
static volatile bool spi2_xfer_done;  /**< DAC-CS1 Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx0_buf[] = TEST_STRING;           /**< ADC TX buffer. */
static uint8_t       m_tx1_buf[] = TEST_STRING;           /**< DAC-CS0 TX buffer. */
static uint8_t       m_tx2_buf[] = TEST_STRING;           /**< DAC-CS1 TX buffer. */
static uint8_t       m_rx0_buf[sizeof(TEST_STRING) + 1];    /**< ADC-RX buffer. */
static uint8_t       m_rx1_buf[sizeof(TEST_STRING) + 1];    /**< DAC-CS0 RX buffer. */
static uint8_t       m_rx2_buf[sizeof(TEST_STRING) + 1];    /**< DAC-CS1 RX buffer. */
static const uint8_t m0_length = sizeof(m_tx0_buf);        /**< ADC TX Transfer length. */
static const uint8_t m1_length = sizeof(m_tx1_buf);        /**< DAC-CS0 Transfer length. */
static const uint8_t m2_length = sizeof(m_tx2_buf);        /**< DAC-CS1 Transfer length. */
