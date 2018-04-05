#ifndef __BT_PROC_CMD_H
#define __BT_PROC_CMD_H

#include <stdio.h>
#include "nrf.h"
#include "bt.h"

#define BT_TX_MSG_SIZE		50

int func_cmd_DARD(unsigned char *data);
int func_cmd_DARC(unsigned char *data);
int func_cmd_DAST(unsigned char *data);
int func_cmd_DASR(unsigned char *data);
int func_cmd_ADSE(unsigned char *data);
int func_cmd_ADmS(unsigned char *data);
int func_cmd_ADnS(unsigned char *data);
int func_cmd_ADKS(unsigned char *data);


typedef enum {
	REQ_DACRD=0,
	REQ_DACRC=1,
	REQ_DACSET=2,
	REQ_DACSETRC=3,
	REQ_ADCSET=4,
	REQ_ADmST=5,
	REQ_ADnST=6,
	REQ_ADKST=7,
	REQ_NO=8,
	REQ_CODE=9,
	REQ_QN=10,
	REQ_CONTAINER=11,
	REQ_WEIGHT=12,
	REQ_READMIN=13,
	REQ_READMAX=14,
	REQ_CONTSET=15,
	REQ_CONTRST=16,
	REQ_ZEROSET=17,
	REQ_CHGNO=18,
	REQ_CHGQN=19,
	REQ_CHGCODE=20,
	REQ_SETHOLD=21,
	REQ_RELHOLD=22,
	REQ_START=23,
	REQ_STOP=24,
	REQ_CHGMIN=25,
	REQ_CHGMAX=26,
#if (WEIGHSCALE == CRWS100)
	REQ_DISP,
	REQ_DEND,
	REQ_MAXSET,
	INF_TEMP,
#endif
	REQ_CMDMAX
} LCcmd_e_type;

typedef struct{
  LCcmd_e_type e;
  int (*func)(unsigned char* data);
} bt_cmd_fet_t;

typedef enum {
	ADC_10M_OHM=0,
	ADC_10_OHM=1,
	ADC_10K_OHM=2
} adc_mode_e;

void bt_proc_response_packet(LCcmd_e_type cmd, unsigned char *data);
extern void spi1_dac_cs0_init(void);
extern void spi1_dac_cs1_init(void);

extern const char *LCcommand[];
extern uint16_t voltage_value[];
extern adc_mode_e nv_adc_mode;
extern bt_cmd_fet_t func_arry[];

#endif
