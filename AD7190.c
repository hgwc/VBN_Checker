/***************************************************************************//**
 *   @file   AD7190.c
 *   @brief  Implementation of AD7190 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 903
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "AD7190.h"     // AD7190 definitions.
//Pawi #include "common.h"
//#include "spi.h"       
//#include "TIME.h"       // TIME definitions.

uint16_t ConvRate = 0x60;

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 * @param modifyCS - Allows Chip Select to be modified.
 *
 * @return none.
*******************************************************************************/
void AD7190_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber,
                             unsigned char modifyCS)
{
#if 0		
    uint16_t writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
    
    writeCommand[0] = AD7190_COMM_WRITE |
                      AD7190_COMM_ADDR(registerAddress);
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = (0x00FF & *dataPointer);
        dataPointer ++;
        bytesNr --;
    }
	uint8_t value[10];
	for(char i = 0; i < bytesNumber + 1; i++)
	{
		value[i*2] = writeCommand[i];
		value[(i*2)+1] = writeCommand[i] >> 8;
	}
    //SPI_Write(AD7190_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);
//		send_spi1_datas(bytesNumber + 1, writeCommand);
//	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, bytesNumber * 2, m_rx_adc_buf, m_adc_length));
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, (bytesNumber + 1) * 2, NULL, 0));	
	nrf_delay_ms(1);
#else
    uint8_t writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
#if 0
NRF_LOG_INFO("AD7190_SetRegisterValue!");	
#endif
	writeCommand[0] = AD7190_COMM_WRITE |
                      AD7190_COMM_ADDR(registerAddress);
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = (0x00FF & *dataPointer);
        dataPointer ++;
        bytesNr --;
    }
/*
	uint8_t value[10];
	for(char i = 0; i < bytesNumber; i++)
	{
		value[i*2] = writeCommand[i];
		value[(i*2)+1] = writeCommand[i] >> 8;
	}
*/
    //SPI_Write(AD7190_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);
//		send_spi1_datas(bytesNumber + 1, writeCommand);
//	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, bytesNumber * 2, m_rx_adc_buf, m_adc_length));
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc,  writeCommand, bytesNumber + 1, NULL, 0));	
	nrf_delay_ms(1);
#endif
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 * @param modifyCS    - Allows Chip Select to be modified.
 *
 * @return buffer - Value of the register.
*******************************************************************************/
unsigned long AD7190_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber,
                                      unsigned char modifyCS)
{
#if 0
		unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
	unsigned char temp[bytesNumber + 1]         = {0x0, };
    unsigned char i               = 0;
    
    registerWord[0] = AD7190_COMM_READ |
                      AD7190_COMM_ADDR(registerAddress);
	
    //SPI_Read(AD7190_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
//    read_spi_datas(registerWord, bytesNumber + 1);
  	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, registerWord, 5, temp, bytesNumber + 1));   
	nrf_delay_ms(1);	
    for(i = 1; i < bytesNumber + 1; i++) 
    {
//원본        buffer = (buffer << 8) + registerWord[i];
        buffer = (buffer << 8) + temp[i];			
    }
    
    return buffer;
#else
		uint8_t registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
	unsigned char temp[4]         = {0x00, };
    unsigned char i               = 0;
#if 0
NRF_LOG_INFO("AD7190_GetRegisterValue!");
#endif
    registerWord[0] = AD7190_COMM_READ |
                      AD7190_COMM_ADDR(registerAddress);
	
    //SPI_Read(AD7190_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
//    read_spi_datas(registerWord, bytesNumber + 1);
  	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, registerWord, 1, temp, bytesNumber + 1));
	nrf_delay_ms(10);
	
    for(i = 1; i < bytesNumber + 1; i++) 
    {
//원본        buffer = (buffer << 8) + registerWord[i];
        buffer = (buffer << 8) + temp[i];			
    }
    
    return buffer;	
#endif
}

/***************************************************************************//**
 * @brief Checks if the AD7190 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/
unsigned char AD7190_Init(void)
{
    unsigned char status = 1;
    unsigned char regVal = 0;

NRF_LOG_INFO("\r\nAD7190_Init!");		
    //SPI_Init(0, 1000000, 1, 0);
//    init_spi1();
//180315 SPI-S
    nrf_drv_spi_config_t spi0_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi0_config.ss_pin   = SPI0_ADC_CS;
    spi0_config.miso_pin = SPI0_ADC_DIN;
    spi0_config.mosi_pin = SPI0_ADC_DOUT;
    spi0_config.sck_pin  = SPI0_ADC_SCLK;
#if 1
	spi0_config.mode  = NRF_DRV_SPI_MODE_3; 	// 변경 시킴 SCK active low, sample on trailing edge of clock.
#endif	
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi0_adc, &spi0_config, spi0_event_handler, NULL));
// non-blocking에서 blocking으로 변환
//	APP_ERROR_CHECK(nrf_drv_spi_init(&spi0_adc, &spi0_config, NULL, NULL));	
	nrf_delay_ms(1);		
    AD7190_Reset();
    /* Allow at least 500 us before accessing any of the on-chip registers. */
    //TIME_DelayMs(1);
    regVal = AD7190_GetRegisterValue(AD7190_REG_ID, 1, 1);
    NRF_LOG_INFO("ID: 0x%x\r\n",regVal);
#if 0    
    if( (regVal & AD7190_ID_MASK) != ID_AD7190)
    {
        status = 0;
    }
#else
    if( (regVal & AD7192_ID_MASK) != ID_AD7192)
    {
        status = 0;
    }
#endif
    return status ;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/
void AD7190_Reset(void)
{
		unsigned long regVal;
#if 0
		uint16_t registerWord[7];
    
    registerWord[0] = 0x0001;
    registerWord[1] = 0x00FF;
    registerWord[2] = 0x00FF;
    registerWord[3] = 0x00FF;
    registerWord[4] = 0x00FF;
    registerWord[5] = 0x00FF;
    registerWord[6] = 0x00FF;
    
    //SPI_Write(AD7190_SLAVE_ID, registerWord, 7);
//		send_spi1_datas(7, registerWord); 

	uint8_t value[14];
	for(char i = 0; i < 7; i++)
	{
		value[i*2] = registerWord[i];
		value[(i*2)+1] = registerWord[i] >> 8;
	}	
//	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, 14, m_rx_adc_buf, m_adc_length));
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, 14, NULL, 0));
	nrf_delay_ms(2);
#if 1
regVal = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
NRF_LOG_INFO("Confiuration( 0x000117) = %x\n", regVal);
nrf_delay_ms(100);
regVal = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
NRF_LOG_INFO("Mode( 0x080060) = %x\n", regVal);
nrf_delay_ms(100);
regVal = AD7190_GetRegisterValue(AD7190_REG_OFFSET, 3, 1);
NRF_LOG_INFO("Offset(0x800000) = %x\n", regVal);
nrf_delay_ms(100);
regVal = AD7190_GetRegisterValue(AD7190_REG_STAT, 1, 1);
NRF_LOG_INFO("Stat( 0x80) = %x\n", regVal);
nrf_delay_ms(100);
#endif
#else
		uint8_t registerByte[7];

NRF_LOG_INFO("\r\nAD7190_Reset!");
    registerByte[0] = 0xFF;
    registerByte[1] = 0xFF;
    registerByte[2] = 0xFF;
    registerByte[3] = 0xFF;
    registerByte[4] = 0xFF;
    registerByte[5] = 0xFF;
    registerByte[6] = 0xFF;	
    
    //SPI_Write(AD7190_SLAVE_ID, registerWord, 7);
//		send_spi1_datas(7, registerWord); 

//	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, value, 14, m_rx_adc_buf, m_adc_length));
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi0_adc, registerByte, 7, NULL, 0));
	nrf_delay_ms(2);
#if 1
regVal = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
NRF_LOG_INFO("Confiuration Initial Value( 0x000117) = %x\n", regVal);
nrf_delay_ms(1);
regVal = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
NRF_LOG_INFO("Mode Initial Value( 0x080060) = %x\n", regVal);
nrf_delay_ms(1);
regVal = AD7190_GetRegisterValue(AD7190_REG_OFFSET, 3, 1);
NRF_LOG_INFO("Offset Initial Value(0x800000) = %x\n", regVal);
nrf_delay_ms(1);
#endif	
#endif
}

/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/
void AD7190_SetPower(unsigned char pwrMode)
{
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
	 unsigned long newPwrModeTemp = 0x0;
 
NRF_LOG_INFO("\r\nAD7190_SetPower!");	 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
     newPwrMode = oldPwrMode | 
                  AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) |
                                  (!pwrMode * (AD7190_MODE_PWRDN)));
	 newPwrModeTemp = newPwrMode;
     AD7190_SetRegisterValue(AD7190_REG_MODE, newPwrMode, 3, 1);
	 newPwrMode = 0x0;
     newPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
	
	 if(newPwrModeTemp == newPwrMode)
	 {
     	NRF_LOG_INFO("AD7190_SetPower! 0x%04x",newPwrMode);
	 } 
	else
	{
		NRF_LOG_INFO("AD7190_SetPower Fail!******");
	}	 
}

/***************************************************************************//**
 * @brief Set Bridge Power on/off.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           	1 - power-on
 *
 * @return none.
*******************************************************************************/
void AD7190_SetBridgePower(unsigned char pwrMode)
{
     uint8_t oldPwrMode = 0x0;
     uint8_t newPwrMode = 0x0; 
	 uint8_t newPwrModeTemp = 0x0;

NRF_LOG_INFO("\r\nAD7190_SetBridgePower!");	 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_GPOCON, 1, 1);
     oldPwrMode &= ~(AD7190_GPOCON_BPDSW);
     newPwrMode = oldPwrMode | (pwrMode * AD7190_GPOCON_BPDSW);

	 newPwrModeTemp = newPwrMode;
#if 0
	 NRF_LOG_INFO("newPwrModeTemp = %d\n", newPwrMode);
#endif
     AD7190_SetRegisterValue(AD7190_REG_GPOCON, newPwrMode, 1, 1);
     
     newPwrMode = 0x0;
     newPwrMode = AD7190_GetRegisterValue(AD7190_REG_GPOCON, 1, 1);
	
	 if(newPwrModeTemp == newPwrMode)
	 {
     	NRF_LOG_INFO("AD7190_SetBridgePower! 0x%02x",newPwrMode);
	 } 
	else
	{
		NRF_LOG_INFO("AD7190_SetBridgePower Fail!******");
	}
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(void)
{
    unsigned long timeOutCnt = 0xFFFFF;
    uint16_t i = 0;

NRF_LOG_INFO("\r\nAD7190_WaitRdyGoLow!");		
    while(timeOutCnt--)
    {
//원본    	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET)
    		i++;
    	if(i > 3)
    		break;
    }
    if(timeOutCnt <= 0)
    	NRF_LOG_INFO("AD Conversion timeout! %d\r\n",timeOutCnt);
}

/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *  
 * @return none.
*******************************************************************************/
void AD7190_ChannelSelect(unsigned short channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

NRF_LOG_INFO("\r\nAD7190_ChannelSelect!");	
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190_MultiChannelSelect(unsigned short chan1, unsigned short chan2)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
 
NRF_LOG_INFO("\r\nAD7190_MultiChannelSelect!");		
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << chan1)| AD7190_CONF_CHAN(1 << chan2);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190_4ChannelSelect(unsigned short chan1, unsigned short chan2, unsigned short chan3, unsigned short chan4)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
 
NRF_LOG_INFO("\r\nAD7190_4ChannelSelect!");		
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << chan1)| AD7190_CONF_CHAN(1 << chan2) \
		| AD7190_CONF_CHAN(1 << chan3) | AD7190_CONF_CHAN(1 << chan4);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief chop enable.
 *
 * @return none.
*******************************************************************************/
uint32_t AD7190_ChopEnable(unsigned char chop)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

NRF_LOG_INFO("\r\nAD7190_ChopEnable!");	
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHOP);
    if(chop)
    	newRegValue = oldRegValue | AD7190_CONF_CHOP;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
    
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    if((oldRegValue & AD7190_CONF_CHOP) && !chop)
    	NRF_LOG_INFO("Chop Disable Failed!******\r\n");
    	
    if(!(oldRegValue & AD7190_CONF_CHOP) && chop)
    	NRF_LOG_INFO("Chop Enable Failed!******\r\n"); 

    return (oldRegValue & AD7190_CONF_CHOP);
}

/***************************************************************************//**
 * @brief Set REFDET.
 *
 * @return none.
*******************************************************************************/
void AD7190_RefDetEnable(unsigned char refdet)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
 
NRF_LOG_INFO("\r\nAD7190_RefDetEnable!");		
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_REFDET);
    if(refdet)
    	newRegValue = oldRegValue | AD7190_CONF_REFDET;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190_RefSelect(unsigned char refsel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

NRF_LOG_INFO("\r\nAD7190_RefSelect!");	
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_REFSEL);
    if(refsel)
    	newRegValue = oldRegValue | AD7190_CONF_REFSEL;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190_BufEnable(unsigned char buf)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
NRF_LOG_INFO("\r\nAD7190_BufEnable!");
	oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_BUF);
    if(buf)
    	newRegValue = oldRegValue | AD7190_CONF_BUF;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Set REJ 60.
 *
 * @return none.
*******************************************************************************/
void AD7190_Rej60Enable(unsigned char rej)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

NRF_LOG_INFO("\r\nAD7190_Rej60Enable!");	
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~(AD7190_MODE_REJ60);
    if(rej)
    	newRegValue = oldRegValue | AD7190_MODE_REJ60;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3, 1);
}

void AD7190_DatSta_Enable(unsigned char datsta)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   

NRF_LOG_INFO("\r\nAD7190_DatSta_Enable!");	
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~(AD7190_MODE_DAT_STA);
    if(datsta)
    	newRegValue = oldRegValue | AD7190_MODE_DAT_STA;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3, 1);
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void AD7190_Calibrate(unsigned char mode, unsigned char channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;

NRF_LOG_INFO("\r\nAD7190_Calibrate!");	
    AD7190_ChannelSelect(channel);
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7190_MODE_SEL(mode);
    //ADI_PART_CS_LOW; 
    AD7190_SetRegisterValue(AD7190_REG_MODE, newRegValue, 3, 0); // CS is not modified.
    AD7190_WaitRdyGoLow();
    //ADI_PART_CS_HIGH;
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *
 * @return none.
*******************************************************************************/
void AD7190_RangeSetup(unsigned char polarity, unsigned char range)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;

NRF_LOG_INFO("\r\nAD7190_RangeSetup!");		
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF,3, 1);
    oldRegValue &= ~(AD7190_CONF_UNIPOLAR |
                     AD7190_CONF_GAIN(0x7));
    newRegValue = oldRegValue | 
                  (polarity * AD7190_CONF_UNIPOLAR) |
                  AD7190_CONF_GAIN(range); 
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
    
    //CurrGain = (1 << range);
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long AD7190_SingleConversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;

NRF_LOG_INFO("\r\nAD7190_SingleConversion!");	
    command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | 
              AD7190_MODE_CLKSRC(AD7190_CLK_INT) |
              AD7190_MODE_RATE(ConvRate);    
    //ADI_PART_CS_LOW;
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.
    AD7190_WaitRdyGoLow();
    regData = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0);
    //ADI_PART_CS_HIGH;
    
    return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long AD7190_ContinuousReadAvg(unsigned char sampleNumber)
{
    unsigned long samplesAverage = 0x0;
    unsigned char count = 0x0;
    unsigned long command = 0x0;
    unsigned long samMin = 0x0;
    unsigned long samMax = 0x0;
    unsigned long samtemp = 0x0;

NRF_LOG_INFO("\r\nAD7190_ContinuousReadAvg!");	
    if((sampleNumber < 4) && (sampleNumber > 1)){
    	NRF_LOG_INFO("Wrong Parameter! %d should be more than 3\r\n",sampleNumber);
    	sampleNumber = 4;
    }
    	
    command = AD7190_MODE_SEL(AD7190_MODE_CONT) | 
              AD7190_MODE_CLKSRC(AD7190_CLK_INT) |
              AD7190_MODE_RATE(ConvRate);
              //AD7190_MODE_RATE(0x060);
    //ADI_PART_CS_LOW;
    AD7190_SetRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.
    
    if(sampleNumber > 1)
    {
	    for(count = 0;count < sampleNumber;count ++)
	    {
	    		
	        AD7190_WaitRdyGoLow();
	        samtemp = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0); // CS is not modified.
	        if(count == 0)
	        	samMin = samMax = samtemp;
	        else
	        {
	        	if(samMin > samtemp)
	        		samMin = samtemp;
	        	if(samMax < samtemp)
	        		samMax = samtemp;
	        }
	        samplesAverage += samtemp;
	    }
	    //ADI_PART_CS_HIGH;
	    //samplesAverage = samplesAverage / sampleNumber;
	    samplesAverage = (samplesAverage -samMin -samMax) / (sampleNumber - 2);
    }
    else
    {
      AD7190_WaitRdyGoLow();
      samtemp = AD7190_GetRegisterValue(AD7190_REG_DATA, 3, 0); // CS is not modified.
      samplesAverage = samtemp;
    }
		AD7190_SetPower(1); //idle
    return samplesAverage ;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
unsigned long AD7190_TemperatureRead(void)
{
    unsigned char temperature = 0x0;
    unsigned long dataReg = 0x0;

NRF_LOG_INFO("\r\nAD7190_TemperatureRead!");	
    AD7190_RangeSetup(0, AD7190_CONF_GAIN_1);
    AD7190_ChannelSelect(AD7190_CH_TEMP_SENSOR);
    dataReg = AD7190_SingleConversion();
    dataReg -= 0x800000;
    dataReg /= 2815;   // Kelvin Temperature
    dataReg -= 273;    //Celsius Temperature
    temperature = (unsigned long) dataReg;
    
    return temperature;
}


/* ///////////////////////////////////////////// */
/* /////////////// Averaging //////////////////// */
/* //////////////////////////////////////////// */

#define MAX_QUE			31
#define MAX_AVER		(MAX_QUE-1)
#define MAX_ROFF		5	

uint32_t	TEMP_QUEUE[MAX_QUE] = {0,};
uint32_t	TempAverage = 0;
uint32_t	ADC_QUEUE[MAX_QUE] = {0,};
uint32_t	AdcAverage = 0;
//uint8_t	U3_QUEUE[MAX_QUEUE];
uint32_t	HIGH_VALT[MAX_AVER] = {0,};
uint32_t	LOW_VALT[MAX_AVER] = {0,};
uint32_t	HIGH_VALA[MAX_AVER] = {0,};
uint32_t	LOW_VALA[MAX_AVER] = {0,};

uint8_t	HighIdx = 0;
uint8_t	LowIdx = 0;
uint8_t	HighIdxA = 0;
uint8_t	LowIdxA = 0;

unsigned int frontt, reart;
unsigned int fronta, reara;
//unsigned int front3, rear3;
extern uint8_t	AverageStartF;

void InitQueueTemp()
{
	frontt =  reart = 0;
	TempAverage = 0;
	HighIdx = 0;
	LowIdx = 0;	
}

void ClearQueueTemp()
{
	frontt =  reart;
	TempAverage = 0;
	HighIdx = 0;
	LowIdx = 0;	
}

uint32_t PutAverQueTemp(uint32_t n)
{
	uint32_t retval = 0;
	uint32_t temp = 0;
	
	if((reart +1) % MAX_QUE != frontt)
	//if(frontt != reart)
	{
		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;
		NRF_LOG_INFO("*temperature: put only 0x%08x, %d\r\n",n,reart);
		return 0;
	}
	else
	{
		TempAverage -= TEMP_QUEUE[frontt];
		//temp = TEMP_QUEUE[frontt];
		TEMP_QUEUE[frontt] = 0;
		frontt++;
		frontt %= MAX_QUE;

		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;

		retval = TempAverage/MAX_AVER;
		NRF_LOG_INFO("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reart,frontt);
	}

	return retval;
}

uint32_t PutAverQueTemp2(uint32_t n)
{
	uint32_t retval = 0;
	uint32_t temp = 0;
	uint8_t i = 0;
	char x = 0;
	uint32_t hightotal = 0;
	uint32_t lowtotal = 0;	

	if((reart +1) % MAX_QUE != frontt)
	//if(frontt != reart)
	{
		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;
		
		NRF_LOG_INFO("*temperature: put only 0x%08x, %d\r\n",n,reart);
		return n;
	}
	else
	{
		TempAverage -= TEMP_QUEUE[frontt];
		//temp = TEMP_QUEUE[frontt];
		TEMP_QUEUE[frontt] = 0;
		frontt++;
		frontt %= MAX_QUE;

		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;

		//uint32_t temp = 0;
		for(i=0; i<MAX_AVER; i++)
		{
			HIGH_VALT[i] = TEMP_QUEUE[(frontt+i)%MAX_QUE];
			LOW_VALT[i] = TEMP_QUEUE[(frontt+i)%MAX_QUE];
		}
		
		for(i=0; i<MAX_ROFF; i++)
		{
			for(x=(i+1); x < MAX_AVER; x++)
			{
				if(HIGH_VALT[i] < HIGH_VALT[x])
				{
					temp = HIGH_VALT[i];
					HIGH_VALT[i] = HIGH_VALT[x];
					HIGH_VALT[x] = temp;
				}
			}
		}

		for(i=0; i<MAX_ROFF; i++)
		{
			for(x=(i+1); x < MAX_AVER; x++)
			{
				if(LOW_VALT[i] > LOW_VALT[x])
				{
					temp = LOW_VALT[i];
					LOW_VALT[i] = LOW_VALT[x];
					LOW_VALT[x] = temp;
				}
			}
		}
	NRF_LOG_INFO("**HIGH_VALT: %d, %d, %d, %d, %d\r\n",HIGH_VALT[0],HIGH_VALT[1],HIGH_VALT[2],HIGH_VALT[3],HIGH_VALT[4]);
	NRF_LOG_INFO("**LOW_VALT: %d, %d, %d, %d, %d\r\n",LOW_VALT[0],LOW_VALT[1],LOW_VALT[2],LOW_VALT[3],LOW_VALT[4]);

		hightotal = HIGH_VALT[0]+HIGH_VALT[1]+HIGH_VALT[2]+HIGH_VALT[3]+HIGH_VALT[4];
		lowtotal = LOW_VALT[0]+LOW_VALT[1]+LOW_VALT[2]+LOW_VALT[3]+LOW_VALT[4];

		retval = (TempAverage - hightotal - lowtotal)/(MAX_AVER-10);
		
		if((AverageStartF & 2) == 0)
			AverageStartF |= 2;
		NRF_LOG_INFO("*temperature: %d, %d, %d, %d, %d\r\n",TempAverage,hightotal,lowtotal,retval,n);
		NRF_LOG_INFO("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reart,frontt);
		NRF_LOG_INFO("*temperature: retval 0x%08x, %d\r\n",retval,retval);
	}

	return retval;
}

void InitQueueAdc()
{
	fronta =  reara = 0;
	AdcAverage = 0;
	HighIdxA = 0;
	LowIdxA = 0;	
}

void ClearQueueAdc()
{
	fronta =  reara;
	AdcAverage = 0;
	HighIdxA = 0;
	LowIdxA = 0;	
}

uint32_t PutAverQueAdc(uint32_t n)
{
	uint32_t retval = 0;
	uint32_t temp = 0;
	
	if((reara +1) % MAX_QUE != fronta)
	//if(frontt != reart)
	{
		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;
		NRF_LOG_INFO("*temperature: put only 0x%08x, %d\r\n",n,reara);
		return 0;
	}
	else
	{
		AdcAverage -= ADC_QUEUE[fronta];
		//temp = TEMP_QUEUE[frontt];
		ADC_QUEUE[fronta] = 0;
		fronta++;
		fronta %= MAX_QUE;

		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;

		retval = AdcAverage/MAX_AVER;
		NRF_LOG_INFO("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reara,fronta);
	}

	return retval;
}

uint32_t PutAverQueAdc2(uint32_t n)
{
	uint32_t retval = 0;
	uint32_t temp = 0;
	uint8_t i = 0;
	char x = 0;
	uint32_t hightotal = 0;
	uint32_t lowtotal = 0;	

	if((reara +1) % MAX_QUE != fronta)
	//if(frontt != reara)
	{
		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;
		
		NRF_LOG_INFO("*temperature: put only 0x%08x, %d\r\n",n,reara);
		return n;
	}
	else
	{
		AdcAverage -= ADC_QUEUE[fronta];
		//temp = TEMP_QUEUE[frontt];
		ADC_QUEUE[fronta] = 0;
		fronta++;
		fronta %= MAX_QUE;

		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;

		//uint32_t temp = 0;
		for(i=0; i<MAX_AVER; i++)
		{
			HIGH_VALA[i] = ADC_QUEUE[(fronta+i)%MAX_QUE];
			LOW_VALA[i] = ADC_QUEUE[(fronta+i)%MAX_QUE];
		}
		
		for(i=0; i<MAX_ROFF; i++)
		{
			for(x=(i+1); x < MAX_AVER; x++)
			{
				if(HIGH_VALA[i] < HIGH_VALA[x])
				{
					temp = HIGH_VALA[i];
					HIGH_VALA[i] = HIGH_VALA[x];
					HIGH_VALA[x] = temp;
				}
			}
		}

		for(i=0; i<MAX_ROFF; i++)
		{
			for(x=(i+1); x < MAX_AVER; x++)
			{
				if(LOW_VALA[i] > LOW_VALA[x])
				{
					temp = LOW_VALA[i];
					LOW_VALA[i] = LOW_VALA[x];
					LOW_VALA[x] = temp;
				}
			}
		}
	NRF_LOG_INFO("**HIGH_VALT: %d, %d, %d, %d, %d\r\n",HIGH_VALT[0],HIGH_VALT[1],HIGH_VALT[2],HIGH_VALT[3],HIGH_VALT[4]);
	NRF_LOG_INFO("**LOW_VALT: %d, %d, %d, %d, %d\r\n",LOW_VALT[0],LOW_VALT[1],LOW_VALT[2],LOW_VALT[3],LOW_VALT[4]);

		hightotal = HIGH_VALA[0]+HIGH_VALA[1]+HIGH_VALA[2]+HIGH_VALA[3]+HIGH_VALA[4];
		lowtotal = LOW_VALA[0]+LOW_VALA[1]+LOW_VALA[2]+LOW_VALA[3]+LOW_VALA[4];

		retval = (AdcAverage - hightotal - lowtotal)/(MAX_AVER-10);

		if((AverageStartF & 1) == 0)
			AverageStartF |= 1;
		NRF_LOG_INFO("*temperature: %d, %d, %d, %d, %d\r\n",TempAverage,hightotal,lowtotal,retval,n);
		NRF_LOG_INFO("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reara,frontt);
		NRF_LOG_INFO("*temperature: retval 0x%08x, %d\r\n",retval,retval);
	}

	return retval;
}
/* ////////////////////////////////////////////// */
