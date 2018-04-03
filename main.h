

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
		 
#include "AD7190.h"		 

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
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define NRF_BLE_GATT_MAX_MTU_SIZE		103			//원본에 업던 것을 추가함

//제어용 GPIO 설정 -S(18.04.02)

#define NO2_PREHEAT	3
#define SHDN		22
//제어용 GPIO 설정 -S(18.04.02)

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
//SPI-E

//ADC7192-S
#if 0
/******************************************************************************/
/*********************** Functions Declarations *******************************/
/******************************************************************************/

/*! Writes data into a register. */
extern void AD7190_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber,
                             unsigned char modifyCS);

/*! Reads the value of a register. */
extern unsigned long AD7190_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber,
                                      unsigned char modifyCS);

/*! Checks if the AD7139 part is present. */
extern unsigned char AD7190_Init(void);

/*! Resets the device. */
extern void AD7190_Reset(void);

/*! Set device to idle or power-down. */
extern void AD7190_SetPower(unsigned char pwrMode);

extern void AD7190_SetBridgePower(unsigned char pwrMode);

/*! Waits for RDY pin to go low. */
extern void AD7190_WaitRdyGoLow(void);

/*! Selects the channel to be enabled. */
extern void AD7190_ChannelSelect(unsigned short channel);

extern void AD7190_MultiChannelSelect(unsigned short chan1, unsigned short chan2);

extern uint32_t AD7190_ChopEnable(unsigned char chop);

extern void AD7190_RefDetEnable(unsigned char refdet);

extern void AD7190_RefSelect(unsigned char refsel);

extern void AD7190_BufEnable(unsigned char buf);

extern void AD7190_Rej60Enable(unsigned char rej);

extern void AD7190_DatSta_Enable(unsigned char datsta);


/*! Performs the given calibration to the specified channel. */
extern void AD7190_Calibrate(unsigned char mode, unsigned char channel);

/*! Selects the polarity of the conversion and the ADC input range. */
extern void AD7190_RangeSetup(unsigned char polarity, unsigned char range);

/*! Returns the result of a single conversion. */
extern unsigned long AD7190_SingleConversion(void);

/*! Returns the average of several conversion results. */
extern unsigned long AD7190_ContinuousReadAvg(unsigned char sampleNumber);

/*! Read data from temperature sensor and converts it to Celsius degrees. */
extern unsigned long AD7190_TemperatureRead(void);
#endif