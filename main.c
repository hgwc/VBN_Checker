/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include "main.h"

#if 0
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
		 
#include "nrf_delay.h"		 

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#if 0				//속도를 높이기 위해 연결 간격을 BLE가 허용하는 최소로 줄임.....#if 0가 원본임
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#else
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#endif
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(200)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define NRF_BLE_GATT_MAX_MTU_SIZE		103			//원본에 업던 것을 추가함


#include "AD7190.h"
#include "bt_proc_cmd.h"

//제어용 GPIO 설정 -S(18.04.02)
#define NO2_PREHEAT		3
#define SHDN						22
//제어용 GPIO 설정 -S(18.04.02)

#define NRF_LOG_USED		1
#define SAADC 					1

BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static void ble_stack_init(void);
void gatt_init(void);
static void advertising_init(void);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
																					//you have the m_ble_nus_max_data_len that is updated after ATT MTU exchange with the central.
																					//In the function gatt_init() in main.c we are trying to increase the ATT MTU to 64.
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

//180321 Data Length Extension-S
#define L2CAP_HDR_LEN                   4                                               /**< L2CAP header length. */
#endif

uint16_t DAC_Value = 10000;
uint16_t dac1;
uint16_t dac2;
char dac_value3 = '1';
char dac_value4 = '0';

const uint8_t LCcmd_len[] = {
	0, /* DARD */
	0, /* DARC */
	0, /* DAST */
	0, /* DASR */
	0, /* ADSE */
	0, /* ADmS */
	0, /* ADnS */
	0, /* ADKS */
	0, /* RSNO */
	0, /* RCNO */
	0, /* RPNO */
	0, /* RTAR */
	0, /* RCWT */
	0, /* RSP1 */
	0, /* RSP2 */
	0, /* WTAR */
	0, /* WTRS */
	0, /* WZER */
	6, /* WSNO */
	2, /* WPNO */
	6, /* WCNO */
	0, /* WHOL */
	0, /* WHRS */
	0, /* WSTR */
	0, /* WSTO */
	6, /* WSP1 */
	6, /* WSP2 */
#if (WEIGHSCALE == CRWS100)
	4, /* WDSP */
	0, /* WDED */
	0, /* WMAX */
	6, /* ITEM */
#endif
	0 /*"" */
};

static void conn_evt_len_ext_set(bool status)
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = status ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}


void data_len_ext_set(bool status)
{
//원본    m_test_params.data_len_ext_enabled = status;

    uint8_t data_length = status ? (247 + L2CAP_HDR_LEN) : (23 + L2CAP_HDR_LEN);
    (void) nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, data_length);
}
//180321 Data Length Extension-E

//180315 SPI-S
#include "nrf_drv_spi.h"
#define SPI0_INSTANCE  0 /**< SPI0 instance index. */
#define SPI1_INSTANCE  1 /**< SPI1 instance index. */
const nrf_drv_spi_t spi0_adc = NRF_DRV_SPI_INSTANCE(SPI0_INSTANCE);  /**< SPI instance. */
const nrf_drv_spi_t spi1_dac = NRF_DRV_SPI_INSTANCE(SPI1_INSTANCE);  /**< SPI instance. */
static volatile bool spi0_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool spi1_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
uint8_t       m_tx_adc_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_tx_dac_buf[] = TEST_STRING;           /**< TX buffer. */
uint8_t       m_rx_adc_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static uint8_t       m_rx_dac_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
const uint8_t m_adc_length = sizeof(m_tx_adc_buf);        /**< Transfer length. */
static const uint8_t m_dac_length = sizeof(m_tx_dac_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi0_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	if(p_event -> type == NRF_DRV_SPI_EVENT_DONE)
	{
		spi0_xfer_done = true;
//		NRF_LOG_INFO("SPI0_Transfer completed.");
    	if (m_rx_adc_buf[0] != 0)
    	{
       		NRF_LOG_INFO(" SPI0_Received:");
        	NRF_LOG_HEXDUMP_INFO(m_rx_adc_buf, strlen((const char *)m_rx_adc_buf));
   	 	}
	}
	else
		NRF_LOG_INFO("SPI Error\n");
}

void spi1_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi1_xfer_done = true;

	NRF_LOG_INFO("SPI1_Transfer completed.");
    if (m_rx_dac_buf[0] != 0)
    {
       NRF_LOG_INFO(" SPI1_Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_dac_buf, strlen((const char *)m_rx_dac_buf));
    }
}
//180315 SPI-E

//180316 SAADC-S
#define SAMPLES_IN_BUFFER         6		//원본 4
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 5000);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);

	nrf_gpio_pin_write(NO2_PREHEAT, 1);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	nrf_gpio_pin_write(NO2_PREHEAT, 0);
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
				uint16_t adc_value;
				uint8_t value[SAMPLES_IN_BUFFER*2];
				uint16_t bytes_to_send;		

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("ADC event number: %d\r\n", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
			if(p_event->data.done.p_buffer[i] & 0x8000)
			{
					p_event->data.done.p_buffer[i] = 0;		//180402 음수가 나오면 0으로 변경				
			}
			
           NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);
	   
		}
		bytes_to_send = sizeof(p_event->data.done.p_buffer[i]);
						adc_value = p_event->data.done.p_buffer[i];
						value[i*2] = adc_value;
						value[(i*2)+1] = adc_value >> 8;		
        do
		{
				err_code = ble_nus_string_send(&m_nus, value, &bytes_to_send);
				if(err_code == 13313)
				{
    				err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
					APP_ERROR_CHECK(err_code);
				}							
                if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                {
						APP_ERROR_CHECK(err_code);					
                }
        } while (err_code == NRF_ERROR_BUSY);			
        m_adc_evt_counter++;
    }
		nrf_gpio_pin_write(NO2_PREHEAT, 1);
}


void saadc_init(void)
{
    ret_code_t err_code;
#if 1
    nrf_saadc_channel_config_t channel_config0 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);		//NO2_Sensor
//		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1);		//defferential로 변경

  nrf_saadc_channel_config_t channel_config2 =
       NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);		//NH3_Sensor
   
  nrf_saadc_channel_config_t channel_config3 =
		  NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);   	//VOC_Sensor
   
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
   	err_code = nrf_drv_saadc_channel_init(2, &channel_config2);	
   	err_code = nrf_drv_saadc_channel_init(3, &channel_config3);		   
#else
    nrf_saadc_channel_config_t channel_config0 =
	//         NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);		//NO2_Sensor
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_DIFFERENTIAL(NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1);		//defferential로 변경

   nrf_saadc_channel_config_t channel_config2 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);		//NH3_Sensor
   
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);
    err_code = nrf_drv_saadc_channel_init(2, &channel_config2);	
#endif

    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
//시험	
	nrf_gpio_cfg_output(NO2_PREHEAT);		//GPIO 3번을 Output으로 설정

}
//180316 SAADC-E

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
//원본                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length-1] == '\r')
        {
//원본            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\n");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

#ifndef S140
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
#if !defined (S112)
         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;
#endif //!defined (S112)
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

 //원본   err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_BLE_GATT_MAX_MTU_SIZE);
	
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
//원본            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_string_send(&m_nus, data_array, &length);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
//    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;		//Advertising T/O이 걸리면 Advertising 중지
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */

static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

	
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

uint16_t voltage_value[] = {
	0, 2621, 5242, 7864, 10485, 13107, //0, 0.1, 0.2, 0.3, 0.4, 0.5
	15728, 18350, 20971, 23592, 26214, //0.6, 0.7, 0.8, 0.9, 1.0
	28835, 31457, 34078, 36700, 39321, //1.1, 1.2, 1.3, 1.4, 1.5
	41943, 44564, 47185, 49807, 52428, //1.6, 1.7, 1.8, 1.9, 2.0
	55050, 57671, 60293, 62914, 65535  //2.1, 2.2, 2.3, 2.4, 2.5
};

void spi1_dac_cs0_init(void)
{
	m_tx_dac_buf[0] = 0x00;
	m_tx_dac_buf[1]= ((voltage_value[10] - 32768) & 0xff00) >> 8;	
	m_tx_dac_buf[2] = (voltage_value[10] - 32768) & 0xff;	
//	uint16_t dac6[3] = {0x00, dac4, dac5};
//	dac_datas(dac6);
	
//    spi1_dac_cs0_init();
     nrf_drv_spi_config_t spi1_config = NRF_DRV_SPI_DEFAULT_CONFIG;
     spi1_config.ss_pin   = SPI1_DAC_CS0;
     spi1_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
     spi1_config.mosi_pin = SPI1_DAC_SDI;
     spi1_config.sck_pin  = SPI1_DAC_SCLK;
//	 APP_ERROR_CHECK(nrf_drv_spi_init(&spi1_dac, &spi1_config, spi1_event_handler, NULL));	
// non-blocking에서 blocking으로 전환	
	 APP_ERROR_CHECK(nrf_drv_spi_init(&spi1_dac, &spi1_config, NULL, NULL));	 
	 nrf_delay_ms(1);
// 	 APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1_dac, m_tx_dac_buf, m_dac_length, NULL, m_dac_length));	 
	nrf_delay_ms(1);
}

void spi1_dac_cs1_init(void)
{
	m_tx_dac_buf[0] = 0x00;
	m_tx_dac_buf[1]= ((voltage_value[10] - 32768) & 0xff00) >> 8;	
	m_tx_dac_buf[2] = (voltage_value[10] - 32768) & 0xff;	
//	uint16_t dac6[3] = {0x00, dac4, dac5};
//	dac_datas(dac6);
     nrf_drv_spi_config_t spi1_config = NRF_DRV_SPI_DEFAULT_CONFIG;
     spi1_config.ss_pin   = SPI1_DAC_CS1;
     spi1_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
     spi1_config.mosi_pin = SPI1_DAC_SDI;
     spi1_config.sck_pin  = SPI1_DAC_SCLK;
//     APP_ERROR_CHECK(nrf_drv_spi_init(&spi1_dac, &spi1_config, spi1_event_handler, NULL));
// non-blocking에서 blocking으로 전환
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi1_dac, &spi1_config, NULL, NULL));	
	 nrf_delay_ms(1);	
//	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1_dac, m_tx_dac_buf, m_dac_length, NULL, m_dac_length));
	nrf_delay_ms(1);
}

void ADC_reinit(void)
{
	unsigned long adcmd = 0;
	if (AD7190_Init() == 0) {
#if NRF_LOG_USED
		NRF_LOG_INFO("ADC_reinit AD7190_Init Failed!\r\n");
		return;		
#else
		uart1_printf("ADC_reinit AD7190_Init Failed!\r\n");
		return;
#endif
	}
 	AD7190_SetBridgePower(1);										// Bridge Down Switch On(connect)
	AD7190_SetPower(1);							  						// idle 즉 power-down 모드가 아님

#if 0
AD7190_MultiChannelSelect(AD7190_CH_AIN1P_AINCOM | AD7190_CH_AIN2P_AINCOM, AD7190_CH_AIN3P_AINCOM | AD7190_CH_AIN4P_AINCOM );
//AD7190_MultiChannelSelect(AD7190_CH_AIN3P_AINCOM,  AD7190_CH_AIN4P_AINCOM);
#else
AD7190_4ChannelSelect(AD7190_CH_AIN1P_AINCOM, AD7190_CH_AIN2P_AINCOM, AD7190_CH_AIN3P_AINCOM, AD7190_CH_AIN4P_AINCOM ); 	// AIN1, COM 각 채널을  pseudo differential mode 선택
//	AD7190_ChannelSelect(AD7190_CH_AIN2P_AINCOM); 	// AIN2, COM
//	AD7190_ChannelSelect(AD7190_CH_AIN3P_AINCOM); 	// AIN3, COM
//	AD7190_ChannelSelect(AD7190_CH_AIN4P_AINCOM); 	// AIN4, COM
#endif
	
//원본 	AD7190_ChopEnable(1);						  
	AD7190_ChopEnable(0);						  						// chop disable
// 	AD7190_RefDetEnable(1); //REFDET enable
	
//원본 	AD7190_RefSelect(0);
	AD7190_RefSelect(1);													// EFIN2를 reference로 선택
	AD7190_BufEnable(1);													// Buffer enable
	AD7190_DatSta_Enable(0);											// DAT_STA disable
	AD7190_Rej60Enable(1); 												// REJ60 enable
	AD7190_RangeSetup(1, 7);											// Unipolar & Gain은 128
	
//원본 	ADI_SYNC_LOW;
//	nrf_delay_us(10);
//원본 	ADI_SYNC_HIGH;
	
	adcmd = AD7190_MODE_SEL(AD7190_MODE_CONT) |
	        AD7190_MODE_CLKSRC(AD7190_CLK_INT) |
	        AD7190_MODE_RATE(ConvRate);
	AD7190_SetRegisterValue(AD7190_REG_MODE, adcmd, 3, 0); // Continuous Conversation Mode & Internal 4.92 MHz clock.
																									// & Filter output data rate select bits.= 0x60(96)
																									// Output Data Rate (50.05 Hz)= ( MCLK (4.92MHz) /024 ) / FS(0x60)
}

void 	device_init(void)
{

	spi1_dac_cs0_init();
	nrf_drv_spi_uninit(&spi1_dac);	
    spi1_dac_cs1_init();

	ADC_reinit();
}

#define LCSTR_LEN 20

uint16_t LCcnt = 0;
uint16_t LCid = 0;
char LCcmdStr[4];
char LCresult[LCSTR_LEN];

//char LCresult1[LCSTR_LEN];
uint8_t LCcmdok = 0;
uint16_t LClen = 0;


bt_cmd_fet_t func_arry[REQ_CMDMAX] = {
	{	REQ_DACRD, func_cmd_DARD},
	{	REQ_DACRC, func_cmd_DARC},
	{	REQ_DACSET, func_cmd_DAST},
	{	REQ_DACSETRC, func_cmd_DASR},
	{	REQ_ADCSET, func_cmd_ADSE},
	{	REQ_ADmST, func_cmd_ADmS},
	{	REQ_ADnST, func_cmd_ADnS},
	{	REQ_ADKST, func_cmd_ADKS},
	{	REQ_NO, NULL},
	{	REQ_CODE, NULL},
	{	REQ_QN, NULL},
	{	REQ_CONTAINER, NULL},
	{	REQ_WEIGHT, NULL},
	{	REQ_READMIN, NULL},
	{	REQ_READMAX, NULL},
	{	REQ_CONTSET, NULL},
	{	REQ_CONTRST, NULL},
	{	REQ_ZEROSET, NULL},
	{	REQ_CHGNO, NULL},
	{	REQ_CHGQN, NULL},
	{	REQ_CHGCODE, NULL},
	{	REQ_SETHOLD, NULL},
	{	REQ_RELHOLD, NULL},
	{	REQ_START, NULL},
	{	REQ_STOP, NULL},
	{	REQ_CHGMIN, NULL},
	{	REQ_CHGMAX, NULL},
#if (WEIGHSCALE == CRWS100)
	{	REQ_DISP, NULL},
	{	REQ_DEND, NULL},
	{	REQ_MAXSET, NULL},
	{	INF_TEMP, NULL},
#endif
};
const char *LCcommand[] = {
	"DARD",
	"DARC",
	"DAST",
	"DASR",
	"ADSE",
	"ADmS",
	"ADnS",
	"ADKS"
	"RSNO",
	"RCNO",
	"RPNO",
	"RTAR",
	"RCWT",
	"RSP1",
	"RSP2",
	"WTAR",
	"WTRS",
	"WZER",
	"WSNO",
	"WPNO",
	"WCNO",
	"WHOL",
	"WHRS",
	"WSTR",
	"WSTO",
	"WSP1",
	"WSP2",
#if (WEIGHSCALE == CRWS100)
	"WDSP",
	"WDED",
	"WMAX",
	"ITEM",
#endif
	""
};

void SendADCmResponse(LCcmd_e_type cmd)
{
	char sendstr[20] = {
		0,
	};
	int i = 0, k = 0;
	uint16_t crc3 = 0;
	int tmpData;
	double tmpData1;
	double tmpData2;
//h 	double I;
	AD7190_WaitRdyGoLow();
	tmpData1 = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0);
	nrf_delay_ms(1000);
	tmpData2 = ((5 * (tmpData1 - 0x813500) / 0x7eaf00)) + 0.051 - 0.0009;
//h 	I = (tmpData2 / 1.8) / (0.01 * 100) * 1000;
	//uart1_printf("tmpData: %f mV\r",tmpData2*1000);
	//uart1_printf("i: %f mA\r\n",I);
	tmpData = (int)(tmpData2 * 10000);
	sendstr[i++] = 'A';
	sendstr[i++] = LCcommand[cmd][0];
	sendstr[i++] = LCcommand[cmd][1];
	sendstr[i++] = LCcommand[cmd][2];
	sendstr[i++] = LCcommand[cmd][3];
	for (k = 1; k < 5; k++) {
		crc3 += sendstr[k];
	}
	sendstr[i++] = (tmpData / 10000) % 10 + '0';
	sendstr[i++] = (tmpData / 1000) % 10 + '0';
	sendstr[i++] = (tmpData / 100) % 10 + '0';
	sendstr[i++] = (tmpData / 10) % 10 + '0';
	sendstr[i++] = tmpData % 10 + '0';
	sendstr[i++] = (crc3 / 100) % 10 + '0';
	sendstr[i++] = (crc3 / 10) % 10 + '0';
	sendstr[i++] = crc3 % 10 + '0';
	sendstr[i++] = 'B';
	Serial1_PutStringSize(sendstr, i);
	uart1_printf("\r\n");
}

void SendADCnResponse(LCcmd_e_type cmd)
{
	char sendstr[20] = {
		0,
	};
	int i = 0, k = 0;
	uint16_t crc3 = 0;
	int tmpData;
	double tmpData1;
	double tmpData2;
//h 	double I;
	AD7190_WaitRdyGoLow();
	tmpData1 = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0);
	nrf_delay_ms(1000);
	tmpData2 = ((5 * (tmpData1 - 8404000)) / 8365900) + 0.012; //10
	//tmpData2 = ((5*(tmpData1-0x813500)/0x7eaf00))+0.012; //10
//h 	I = (tmpData2 * 1.98) / (10 * 100) * 1000; //10
	//uart1_printf("tmpData: %f mV\r",tmpData2*1000); //10
	//uart1_printf("i: %f mA\r\n",I); //10
	tmpData = (int)(tmpData2 * 10000);
	sendstr[i++] = 'A';
	sendstr[i++] = LCcommand[cmd][0];
	sendstr[i++] = LCcommand[cmd][1];
	sendstr[i++] = LCcommand[cmd][2];
	sendstr[i++] = LCcommand[cmd][3];
	for (k = 1; k < 5; k++) {
		crc3 += sendstr[k];
	}
	sendstr[i++] = (tmpData / 10000) % 10 + '0';
	sendstr[i++] = (tmpData / 1000) % 10 + '0';
	sendstr[i++] = (tmpData / 100) % 10 + '0';
	sendstr[i++] = (tmpData / 10) % 10 + '0';
	sendstr[i++] = tmpData % 10 + '0';
	sendstr[i++] = (crc3 / 100) % 10 + '0';
	sendstr[i++] = (crc3 / 10) % 10 + '0';
	sendstr[i++] = crc3 % 10 + '0';
	sendstr[i++] = 'B';
	Serial1_PutStringSize(sendstr, i);
	uart1_printf("\r\n");
}

void SendADCKResponse(LCcmd_e_type cmd)
{
	char sendstr[20] = {
		0,
	};
	int i = 0, k = 0;
	uint16_t crc3 = 0;
	int tmpData;
	double tmpData1;
	double tmpData2;
//h 	double I;
	AD7190_WaitRdyGoLow();
	tmpData1 = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0);
	nrf_delay_ms(1000);
	tmpData2 = ((5 * (tmpData1 - 0x813500)) / 0x7eaf00) + 0.050; //10k
//h 	I = tmpData2 / (10000 * 100) * 10000000;					 //10k
	//uart1_printf("tmpData: %f mV\r",tmpData2*1000); //10k
	//uart1_printf("i: %f uA\r\n",I); //10k
	tmpData = (int)(tmpData2 * 10000);
	sendstr[i++] = 'A';
	sendstr[i++] = LCcommand[cmd][0];
	sendstr[i++] = LCcommand[cmd][1];
	sendstr[i++] = LCcommand[cmd][2];
	sendstr[i++] = LCcommand[cmd][3];
	for (k = 1; k < 5; k++) {
		crc3 += sendstr[k];
	}
	sendstr[i++] = (tmpData / 10000) % 10 + '0';
	sendstr[i++] = (tmpData / 1000) % 10 + '0';
	sendstr[i++] = (tmpData / 100) % 10 + '0';
	sendstr[i++] = (tmpData / 10) % 10 + '0';
	sendstr[i++] = tmpData % 10 + '0';
	sendstr[i++] = (crc3 / 100) % 10 + '0';
	sendstr[i++] = (crc3 / 10) % 10 + '0';
	sendstr[i++] = crc3 % 10 + '0';
	sendstr[i++] = 'B';
	Serial1_PutStringSize(sendstr, i);
	uart1_printf("\r\n");
}

void SendSetcmdResponse(LCcmd_e_type cmd, char *val)
{
	char sendstr[20] = {
		0,
	};
	int i = 0, j = 0, k = 0;
	uint16_t crc3 = 0;
	sendstr[i++] = 'A';
	sendstr[i++] = LCcommand[cmd][0];
	sendstr[i++] = LCcommand[cmd][1];
	sendstr[i++] = LCcommand[cmd][2];
	sendstr[i++] = LCcommand[cmd][3];
	for (k = 1; k < 5; k++) {
		crc3 += sendstr[k];
	}
	for (j = 0; j < 5; j++) {
		sendstr[i++] = *val++;
	}
	sendstr[i++] = (crc3 / 100) % 10 + '0';
	sendstr[i++] = (crc3 / 10) % 10 + '0';
	sendstr[i++] = crc3 % 10 + '0';
	sendstr[i++] = 'B';
	Serial1_PutStringSize(sendstr, i);
	nrf_delay_ms(8);
}

void SendReadcmdResponse(LCcmd_e_type cmd, char val1, char val2)
{
	char sendstr[20] = {
		0,
	};
	int i = 0, k = 0;
	uint16_t crc3 = 0;
	sendstr[i++] = 'A';
	sendstr[i++] = LCcommand[cmd][0];
	sendstr[i++] = LCcommand[cmd][1];
	sendstr[i++] = LCcommand[cmd][2];
	sendstr[i++] = LCcommand[cmd][3];
	for (k = 1; k < 5; k++) {
		crc3 += sendstr[k];
	}
	sendstr[i++] = val1;
	sendstr[i++] = val2;
	sendstr[i++] = '0';
	sendstr[i++] = '0';
	sendstr[i++] = '0';
	sendstr[i++] = (crc3 / 100) % 10 + '0';
	sendstr[i++] = (crc3 / 10) % 10 + '0';
	sendstr[i++] = crc3 % 10 + '0';
	sendstr[i++] = 'B';
	Serial1_PutStringSize(sendstr, i);
	//uart1_printf("\rdac voltage= %c.%c V\r\n",val1,val2);
	nrf_delay_ms(8);
}

void RcvCmd(void)
{
	uint8_t rxd1;
//h 	char sign;
	if (rx1_flag) {
		rx1_flag = 0;
		//rx3_enter--;
		LCcmdok = 0;
		LCcnt = 0;
		LClen = 0;
		
//h 		uint8_t LCtail = 0;
		//SerialPutChar1('[');
		if (rx1_enter) {
			//패킷이 정상적으로 들어오면 rx1_enter가 set되어진다
			//rx1_enter--;
			while ((rxd1 = GetByte1()) != 0xff) { //Queue가 끝나기 전까지 검사
				//SerialPutChar1(rxd3);
				if (rxd1 == 0x00) {
					uart1_printf("*uart error 0x00\r\n");
					LCcnt = 0;
				}
				switch (LCcnt) { //초기 LCcnt의 값은 0
				case 0:
					//LCid += (rxd1-'0') * 10;
					//'0'은 asc2로 환산하면 0x30
					// 1~9까지의 asc2는 0x31~0x39이므로
					//0x30을 지우기 위하여 -'0'을 수행한다
					//10의 자리수를 나타내기 위해 10을 곱한다
					break;
				case 1:
					LCcmdStr[0] = rxd1; //명령어를 저장한다
					break;
				//LCid += rxd1-'0';
				//break;
				case 2:
					LCcmdStr[1] = rxd1;
					break;
				case 3:
					LCcmdStr[2] = rxd1;
					break;
				case 4:
					LCcmdStr[3] = rxd1;
					break;
				default:
					if (rxd1 == 'B') { //패킷의  Tail부분에 도달했으면
						LCcmdok = 1;	 //LCcmdok를 1로 set해 패킷전송이 끝났음을 알린다
						if (rx1_enter)   //패킷이 정상적으로 들어온것이 확인되면
							rx1_enter--; //rxX_enter를 0으로 줄여 reset시킨다
					}
					else
						LCresult[LClen++] = rxd1; //패킷의 Tail부분에 도달하지 않았으면
					//LCresult배열에 패킷을 저장시킨다
					//패킷의 data부분
					break;
				}
				LCcnt++; //LCcnt를 1씩 증가시킨다
				if (LCcnt >= 0xffff) {
					uart1_printf("*LCcnt over %d\r\n", LCcnt);
					LCcnt = 0;
				}
				if (LCcmdok) { //패킷전송이 완료되면
					if ((strncmp(LCcmdStr, LCcommand[REQ_DACSET], 4) == 0) && (LClen == LCcmd_len[REQ_DACSET])) {
						//LCcmdStr과 LCcommand를 서로 비교한다
						//uart1_printf("*cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0; //if가 true라면 LCcmdok은 reset
						LCcnt = 0;   //LCcnt도 reset된다
						//break;
					}
					else if ((strncmp(LCcmdStr, LCcommand[REQ_DACRD], 4) == 0) && (LClen == LCcmd_len[REQ_DACRD])) {
						//uart1_printf("cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0;
						LCcnt = 0;
						//PafiEventSet(EVENT_ZEROSET);
						//break;
					}
					else if ((strncmp(LCcmdStr, LCcommand[REQ_DACRD], 4) == 0) && (LClen == LCcmd_len[REQ_DACRD])) {
						//uart1_printf("cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0;
						LCcnt = 0;
						//PafiEventSet(EVENT_ZEROSET);
						//break;
					}
					else if ((strncmp(LCcmdStr, LCcommand[REQ_DACRC], 4) == 0) && (LClen == LCcmd_len[REQ_DACRC])) {
						//uart1_printf("cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0;
						LCcnt = 0;
						//PafiEventSet(EVENT_ZEROSET);
						//break;
					}
					else if ((strncmp(LCcmdStr, LCcommand[REQ_ADCSET], 4) == 0) && (LClen == LCcmd_len[REQ_ADCSET])) {
						//uart1_printf("cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0;
						LCcnt = 0;
						//PafiEventSet(EVENT_ZEROSET);
						//break;
					}
					/*else if((strncmp(LCcmdStr,LCcommand[REQ_MAXSET],4) == 0) && (LClen == LCcmd_len[REQ_MAXSET]))
					{
						uart1_printf("cmd ok! len:%d, %s\r\n",LClen, LCcmdStr);
						LCcmdok = 0;
						LCcnt = 0;
						PafiEventSet(EVENT_MAXSET);
						//break;
					}*/
					else
						LClen = 0; //if가 false라면 LClen을 0으로 초기화
					//uart1_printf("cmd not ok! len:%d, %s\r\n",LClen, LCcmdStr);
					//////////////////////////////////////////////////////////
					LCcmdok = 0; //LCcmdok초기화
					LCcnt = 0;   //LCcnt초기화
					if (rx1_enter)
						rx1_enter--; //rx3_enter초기화
				}
				if (LCcnt > 50)
					uart1_printf("*LCcnt Overflow! %d, %s\r\n", LCcnt, LCcmdStr);
			}
			//SerialPutChar1(']');
//h			uint16_t dac_value1 = 1;
//h			uint16_t dac_value2 = 0;
//h			uint16_t CRC7 = 0;
			uint16_t CRC6 = 0;
			uint16_t CRC5 = 0;
			uint16_t CRC4 = 0;
//h			uint16_t CRC3 = 0;
//h			uint16_t CRC2 = 0;
			uint16_t CRC1 = 0;
			uint16_t CRC0 = 0;
			uint16_t ADC_Value = 0;
			if (strncmp(LCcmdStr, LCcommand[REQ_DACSET], 4) == 0) {
				// voltage데이터 수신,data비교
				DAC_Value = 0;
				DAC_Value += (LCresult[0] - '0') * 10000;
				DAC_Value += (LCresult[1] - '0') * 1000;
				DAC_Value += (LCresult[2] - '0') * 100;
				DAC_Value += (LCresult[3] - '0') * 10;
				DAC_Value += (LCresult[4] - '0');
				CRC0 += (LCresult[5] - '0') * 100;
				CRC0 += (LCresult[6] - '0') * 10;
				CRC0 += (LCresult[7] - '0');
				for (int i = 0; i < 4; i++) {
					CRC1 += LCcmdStr[i];
				}
				if (CRC1 == CRC0) {
					//uart1_printf("cmd OK\r\n");
					DAC_Value = DAC_Value / 1000;
					//DAC_Value = (int)(1000000*DAC_Value)/38.147;
					//  uart1_printf("Setting_Voltage: %c.%c V\n\r",LCresult[0],LCresult[1]);
					dac1 = ((voltage_value[DAC_Value] - 32768) & 0xff00) >> 8;
					dac2 = (voltage_value[DAC_Value] - 32768) & 0xff;
					uint16_t dac3[3] = {0x00, dac1, dac2};
//					dac_datas(dac3);
   					spi1_dac_cs0_init();
					uint8_t value[6];
					uint8_t rcv_value[6];
					for(char i = 0; i < 3; i++)
					{
						value[i*2] = dac3[i];
						value[(i*2)+1] = dac3[i] >> 8;
					}	
 					APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1_dac, value, 6, rcv_value, 7));
	
					SendSetcmdResponse(REQ_DACSETRC, LCresult);
					dac_value3 = LCresult[0];
					dac_value4 = LCresult[1];
				}
				else {
					uart1_printf("cmd error\r\n");
				}
			}
			if (strncmp(LCcmdStr, LCcommand[REQ_DACRD], 4) == 0) {
				// voltage데이터 수신,data비교
				DAC_Value = 0;
				DAC_Value += (LCresult[0] - '0') * 10000;
				DAC_Value += (LCresult[1] - '0') * 1000;
				DAC_Value += (LCresult[2] - '0') * 100;
				DAC_Value += (LCresult[3] - '0') * 10;
				DAC_Value += (LCresult[4] - '0');
				if (DAC_Value != 65535) {
					uart1_printf("packet error(data)\r\n");
					rx1_enter = 0;
				}
				CRC4 += (LCresult[5] - '0') * 100;
				CRC4 += (LCresult[6] - '0') * 10;
				CRC4 += (LCresult[7] - '0');
				for (int i = 0; i < 4; i++) {
					CRC5 += LCcmdStr[i];
				}
				if (CRC5 == CRC4) {
					//uart1_printf("cmd OK\r\n");
					SendReadcmdResponse(REQ_DACRC, dac_value3, dac_value4);
				}
				else {
					uart1_printf("cmd error\r\n");
				}
			}
			if (strncmp(LCcmdStr, LCcommand[REQ_ADCSET], 4) == 0) {
				// voltage데이터 수신,data비교
				ADC_Value = 0;
				ADC_Value += (LCresult[0] - '0') * 10000;
				ADC_Value += (LCresult[1] - '0') * 1000;
				ADC_Value += (LCresult[2] - '0') * 100;
				ADC_Value += (LCresult[3] - '0') * 10;
				ADC_Value += (LCresult[4] - '0');
				if (ADC_Value > 2) {
					uart1_printf("packet error(data)\r\n");
					rx1_enter = 0;
				}
				CRC5 += (LCresult[5] - '0') * 100;
				CRC5 += (LCresult[6] - '0') * 10;
				CRC5 += (LCresult[7] - '0');
				for (int i = 0; i < 4; i++) {
					CRC6 += LCcmdStr[i];
				}
				if (CRC6 == CRC5) {
					//uart1_printf("cmd OK\r\n");
					if (ADC_Value == 0) {
						//uart1_printf("shunt resistor: 10m ohm\r\n");
						while (1) {
							SendADCmResponse(REQ_ADmST);
							RcvCmd();
						}
					}
					else if (ADC_Value == 1) {
						//uart1_printf("shunt resistor: 10 ohm\r\n");
						while (1) {
							SendADCnResponse(REQ_ADnST);
							RcvCmd();
						}
					}
					else if (ADC_Value == 2) {
						//uart1_printf("shunt resistor: 10K ohm\r\n");
						while (1) {
							SendADCKResponse(REQ_ADKST);
							RcvCmd();
						}
					}
				}
				else {
					uart1_printf("cmd error\r\n");
				}
			}
			////////////////////////////////////////////////////////////////////////////////////
			//uart1_printf("*TempValue: %d, %03.02f\r\n",TempValue, (float)((TempValue/100)+((TempValue%100)*0.01)));
		}
	}
}

/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
	bool     erase_bonds;

    // Initialize.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

//원본    uart_init();
    log_init();

	buttons_leds_init(&erase_bonds);

    ble_stack_init();
	gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

//180321 DLE-S		//속도를 높이기 위해 
    data_len_ext_set(1);		//m_test_params.data_len_ext_enabled
    conn_evt_len_ext_set(1);		//m_test_params.conn_evt_len_ext_enabled
//180321 DLE-E

	
//180315 SPI-S
//h  nrf_drv_spi_config_t spi0_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//h  spi0_config.ss_pin   = SPI0_ADC_CS;
//h  spi0_config.miso_pin = SPI0_ADC_DIN;
//h  spi0_config.mosi_pin = SPI0_ADC_DOUT;
//h  spi0_config.sck_pin  = SPI0_ADC_SCLK;
//h  APP_ERROR_CHECK(nrf_drv_spi_init(&spi0_adc, &spi0_config, spi0_event_handler, NULL));

//     nrf_drv_spi_config_t spi1_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//     spi1_config.ss_pin   = SPI1_DAC_CS0;
//     spi1_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
//     spi1_config.mosi_pin = SPI1_DAC_SDI;
//     spi1_config.sck_pin  = SPI1_DAC_SCLK;
//     APP_ERROR_CHECK(nrf_drv_spi_init(&spi1_dac, &spi1_config, spi1_event_handler, NULL));	
	
//180315 SPI-E
	
//180316 SAADC-S
#if SAADC
	NRF_LOG_INFO("SAADC HAL simple example.");
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
#endif
//180316 SAADC-E
	
    printf("\r\nUART Start!\r\n");
    NRF_LOG_INFO("UART Start!");
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

	device_init();
		
    while (1)
    {
//180315 SPI-S			
        // Reset rx buffer and transfer done flag
        memset(m_rx_adc_buf, 0, m_adc_length);
        memset(m_rx_dac_buf, 0, m_dac_length);		

        spi0_xfer_done = false;
//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, m_tx_adc_buf, m_adc_length, m_rx_adc_buf, m_adc_length));
		
		nrf_delay_ms(1000);
        spi1_xfer_done = false;
//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1_dac, m_tx_dac_buf, m_dac_length, m_rx_dac_buf, m_dac_length));
		
//180315 SPI-E

//        while (!spi0_xfer_done)
//        {
//            __WFE();
//        }

        NRF_LOG_FLUSH();

//h		bsp_board_led_invert(BSP_BOARD_LED_0);
//h		bsp_board_led_invert(BSP_BOARD_LED_1);
//h		bsp_board_led_invert(BSP_BOARD_LED_2);		
//h        bsp_board_led_invert(BSP_BOARD_LED_3);		
//h        nrf_delay_ms(200);
    }
//180223 SPI-E	

#if 0
    // Enter main loop.
    for (;;)
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    printf("\r\nUART Start!\r\n");
    NRF_LOG_INFO("UART Start!");		
        power_manage();
    }
#endif	
}


/**
 * @}
 */
