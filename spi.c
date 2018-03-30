
#include "spi.h"

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi0_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi0_xfer_done = true;

//원본	NRF_LOG_INFO("SPI0_Transfer completed.");
    if (m_rx0_buf[0] != 0)
    {
       NRF_LOG_INFO(" SPI0_Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx0_buf, strlen((const char *)m_rx0_buf));
    }
}

void spi1_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi1_xfer_done = true;

//원본	NRF_LOG_INFO("SPI1_Transfer completed.");
    if (m_rx1_buf[0] != 0)
    {
       NRF_LOG_INFO(" SPI1_Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx1_buf, strlen((const char *)m_rx1_buf));
    }
}

void spi2_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi2_xfer_done = true;

//원본	NRF_LOG_INFO("SPI1_Transfer completed.");
    if (m_rx1_buf[0] != 0)
    {
       NRF_LOG_INFO(" SPI2_Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx2_buf, strlen((const char *)m_rx1_buf));
    }
}

void spi_init(void)
{
    nrf_drv_spi_config_t spi0_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi0_config.ss_pin   = ADC_CS;
    spi0_config.miso_pin = ADC_DIN;
    spi0_config.mosi_pin = ADC_DOUT;
    spi0_config.sck_pin  = ADC_SCLK;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi0, &spi0_config, spi0_event_handler, NULL));
	
    nrf_drv_spi_config_t spi1_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi1_config.ss_pin   = DAC_CS0;
//사용하지 않음	   spi1_config.miso_pin = SPI1_MISO_PIN;
    spi1_config.mosi_pin = DAC_SDI;
    spi1_config.sck_pin  = DAC_SCLK;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi1, &spi1_config, spi1_event_handler, NULL));	

    nrf_drv_spi_config_t spi2_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi1_config.ss_pin   = DAC_CS1;
//사용하지 않음	   spi1_config.miso_pin = SPI1_MISO_PIN;
    spi1_config.mosi_pin = DAC_SDI;
    spi1_config.sck_pin  = DAC_SCLK;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi1, &spi2_config, spi2_event_handler, NULL));	
}

