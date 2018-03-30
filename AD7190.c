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
#include "common.h"
//#include "spi.h"       
//#include "TIME.h"       // TIME definitions.

u16 ConvRate = 0x60;

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
    u16 writeCommand[5] = {0, 0, 0, 0, 0};
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
    //SPI_Write(AD7190_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);
		send_spi1_datas(bytesNumber + 1, writeCommand);
		
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
    unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
    unsigned char i               = 0;
    
    registerWord[0] = AD7190_COMM_READ |
                      AD7190_COMM_ADDR(registerAddress);
    //SPI_Read(AD7190_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);
    read_spi_datas(registerWord, bytesNumber + 1);
    
    for(i = 1; i < bytesNumber + 1; i++) 
    {
        buffer = (buffer << 8) + registerWord[i];
    }
    
    return buffer;
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
    
    //SPI_Init(0, 1000000, 1, 0);
    init_spi1();
    AD7190_Reset();
    /* Allow at least 500 us before accessing any of the on-chip registers. */
    //TIME_DelayMs(1);
    delay_ms(2);
    regVal = AD7190_GetRegisterValue(AD7190_REG_ID, 1, 1);
    //uart1_printf("ID: 0x%x\r\n",regVal);
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
    u16 registerWord[7];
    
    registerWord[0] = 0x0001;
    registerWord[1] = 0x00FF;
    registerWord[2] = 0x00FF;
    registerWord[3] = 0x00FF;
    registerWord[4] = 0x00FF;
    registerWord[5] = 0x00FF;
    registerWord[6] = 0x00FF;
    
    //SPI_Write(AD7190_SLAVE_ID, registerWord, 7);
		send_spi1_datas(7, registerWord); 
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
 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7190_MODE_SEL(0x7));
     newPwrMode = oldPwrMode | 
                  AD7190_MODE_SEL((pwrMode * (AD7190_MODE_IDLE)) |
                                  (!pwrMode * (AD7190_MODE_PWRDN)));
     AD7190_SetRegisterValue(AD7190_REG_MODE, newPwrMode, 3, 1);
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
     u8 oldPwrMode = 0x0;
     u8 newPwrMode = 0x0; 
 
     oldPwrMode = AD7190_GetRegisterValue(AD7190_REG_GPOCON, 1, 1);
     oldPwrMode &= ~(AD7190_GPOCON_BPDSW);
     newPwrMode = oldPwrMode | (pwrMode * AD7190_GPOCON_BPDSW);

     AD7190_SetRegisterValue(AD7190_REG_GPOCON, newPwrMode, 1, 1);
     
     newPwrMode = 0x0;
     newPwrMode = AD7190_GetRegisterValue(AD7190_REG_GPOCON, 1, 1);

     //uart1_printf("AD7190_SetBridgePower! 0x%02x\r\n",newPwrMode);
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void AD7190_WaitRdyGoLow(void)
{
    unsigned long timeOutCnt = 0xFFFFF;
    u16 i = 0;
    
    while(timeOutCnt--)
    {
    	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET)
    		i++;
    	if(i > 3)
    		break;
    }
    if(timeOutCnt <= 0)
    	uart1_printf("AD Conversion timeout! %d\r\n",timeOutCnt);
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
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << channel);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190_MultiChannelSelect(unsigned short chan1, unsigned short chan2)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << chan1)| AD7190_CONF_CHAN(1 << chan2);   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}


/***************************************************************************//**
 * @brief chop enable.
 *
 * @return none.
*******************************************************************************/
u32 AD7190_ChopEnable(unsigned char chop)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHOP);
    if(chop)
    	newRegValue = oldRegValue | AD7190_CONF_CHOP;
    else
    	newRegValue = oldRegValue;   
    AD7190_SetRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
    
    oldRegValue = AD7190_GetRegisterValue(AD7190_REG_CONF, 3, 1);
    if((oldRegValue & AD7190_CONF_CHOP) && !chop)
    	uart1_printf("Chop Disable Failed!******\r\n");
    	
    if(!(oldRegValue & AD7190_CONF_CHOP) && chop)
    	uart1_printf("Chop Enable Failed!******\r\n"); 

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

    if((sampleNumber < 4) && (sampleNumber > 1)){
    	uart1_printf("Wrong Parameter! %d should be more than 3\r\n",sampleNumber);
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

u32	TEMP_QUEUE[MAX_QUE] = {0,};
u32	TempAverage = 0;
u32	ADC_QUEUE[MAX_QUE] = {0,};
u32	AdcAverage = 0;
//u8	U3_QUEUE[MAX_QUEUE];
u32	HIGH_VALT[MAX_AVER] = {0,};
u32	LOW_VALT[MAX_AVER] = {0,};
u32	HIGH_VALA[MAX_AVER] = {0,};
u32	LOW_VALA[MAX_AVER] = {0,};

u8	HighIdx = 0;
u8	LowIdx = 0;
u8	HighIdxA = 0;
u8	LowIdxA = 0;

unsigned int frontt, reart;
unsigned int fronta, reara;
//unsigned int front3, rear3;
extern u8	AverageStartF;

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

u32 PutAverQueTemp(u32 n)
{
	u32 retval = 0;
	u32 temp = 0;
	
	if((reart +1) % MAX_QUE != frontt)
	//if(frontt != reart)
	{
		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;
		uart1_printf("*temperature: put only 0x%08x, %d\r\n",n,reart);
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
		uart1_printf("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reart,frontt);
	}

	return retval;
}

u32 PutAverQueTemp2(u32 n)
{
	u32 retval = 0;
	u32 temp = 0;
	u8 i = 0;
	char x = 0;
	u32 hightotal = 0;
	u32 lowtotal = 0;	

	if((reart +1) % MAX_QUE != frontt)
	//if(frontt != reart)
	{
		TEMP_QUEUE[reart] = n;
		TempAverage += n;
		reart++;
		reart %= MAX_QUE;
		
		//uart1_printf("*temperature: put only 0x%08x, %d\r\n",n,reart);
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

		//u32 temp = 0;
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
	//uart1_printf("**HIGH_VALT: %d, %d, %d, %d, %d\r\n",HIGH_VALT[0],HIGH_VALT[1],HIGH_VALT[2],HIGH_VALT[3],HIGH_VALT[4]);
	//uart1_printf("**LOW_VALT: %d, %d, %d, %d, %d\r\n",LOW_VALT[0],LOW_VALT[1],LOW_VALT[2],LOW_VALT[3],LOW_VALT[4]);

		hightotal = HIGH_VALT[0]+HIGH_VALT[1]+HIGH_VALT[2]+HIGH_VALT[3]+HIGH_VALT[4];
		lowtotal = LOW_VALT[0]+LOW_VALT[1]+LOW_VALT[2]+LOW_VALT[3]+LOW_VALT[4];

		retval = (TempAverage - hightotal - lowtotal)/(MAX_AVER-10);
		
		if((AverageStartF & 2) == 0)
			AverageStartF |= 2;
		//uart1_printf("*temperature: %d, %d, %d, %d, %d\r\n",TempAverage,hightotal,lowtotal,retval,n);
		//uart1_printf("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reart,frontt);
		//uart1_printf("*temperature: retval 0x%08x, %d\r\n",retval,retval);
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

u32 PutAverQueAdc(u32 n)
{
	u32 retval = 0;
	u32 temp = 0;
	
	if((reara +1) % MAX_QUE != fronta)
	//if(frontt != reart)
	{
		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;
		uart1_printf("*temperature: put only 0x%08x, %d\r\n",n,reara);
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
		uart1_printf("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reara,fronta);
	}

	return retval;
}

u32 PutAverQueAdc2(u32 n)
{
	u32 retval = 0;
	u32 temp = 0;
	u8 i = 0;
	char x = 0;
	u32 hightotal = 0;
	u32 lowtotal = 0;	

	if((reara +1) % MAX_QUE != fronta)
	//if(frontt != reara)
	{
		ADC_QUEUE[reara] = n;
		AdcAverage += n;
		reara++;
		reara %= MAX_QUE;
		
		//uart1_printf("*temperature: put only 0x%08x, %d\r\n",n,reara);
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

		//u32 temp = 0;
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
	//uart1_printf("**HIGH_VALT: %d, %d, %d, %d, %d\r\n",HIGH_VALT[0],HIGH_VALT[1],HIGH_VALT[2],HIGH_VALT[3],HIGH_VALT[4]);
	//uart1_printf("**LOW_VALT: %d, %d, %d, %d, %d\r\n",LOW_VALT[0],LOW_VALT[1],LOW_VALT[2],LOW_VALT[3],LOW_VALT[4]);

		hightotal = HIGH_VALA[0]+HIGH_VALA[1]+HIGH_VALA[2]+HIGH_VALA[3]+HIGH_VALA[4];
		lowtotal = LOW_VALA[0]+LOW_VALA[1]+LOW_VALA[2]+LOW_VALA[3]+LOW_VALA[4];

		retval = (AdcAverage - hightotal - lowtotal)/(MAX_AVER-10);

		if((AverageStartF & 1) == 0)
			AverageStartF |= 1;
		//uart1_printf("*temperature: %d, %d, %d, %d, %d\r\n",TempAverage,hightotal,lowtotal,retval,n);
		//uart1_printf("*temperature: Aver <==0x%08x, 0x%08x==>, %d, %d\r\n",n,temp,reara,frontt);
		//uart1_printf("*temperature: retval 0x%08x, %d\r\n",retval,retval);
	}

	return retval;
}
/* ////////////////////////////////////////////// */
