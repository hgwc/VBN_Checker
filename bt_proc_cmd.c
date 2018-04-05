
//#include "bt.h"
#include "bt_proc_cmd.h"

int func_cmd_DARD(unsigned char *data);
int func_cmd_DARC(unsigned char *data);
int func_cmd_DAST(unsigned char *data);
int func_cmd_DASR(unsigned char *data);
int func_cmd_ADSE(unsigned char *data);
int func_cmd_ADmS(unsigned char *data);
int func_cmd_ADnS(unsigned char *data);
int func_cmd_ADKS(unsigned char *data);

unsigned char bt_tx_msg[BT_TX_MSG_SIZE];

//nv
adc_mode_e nv_adc_mode = ADC_10_OHM;
int nv_dac_voltage = 10000;

/*
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
*/

int strtoint_n(unsigned char *str, int n)
{
	int sign = 1;
	int place = 1;
	int ret = 0;
	int i;
	for (i = n - 1; i >= 0; i--, place *= 10) {
		int c = str[i];
		switch (c) {
		case '-':
			if (i == 0) sign = -1;
			else return -1;
			break;
		default:
			if (c >= '0' && c <= '9')   ret += (c - '0') * place;
			else return -1;
		}
	}
	return sign * ret;
}

int func_cmd_DARD(unsigned char *data)
{
	int set_value = strtoint_n(data, BT_PKT_DATA_SIZE);
	if (set_value != 65535) {
		//h ERR("DARD packet error(data)\r\n");
		rx1_enter = 0;
	}
	//response dac voltage
	bt_proc_response_packet(REQ_DACRC, (unsigned char *)&nv_dac_voltage);
	return 0;		//h가 추가
}

int func_cmd_DARC(unsigned char *data)
{
	int bt_tx_msg_len;
	memset(bt_tx_msg, 0, sizeof(bt_tx_msg));
	bt_tx_msg_len = snprintf((char *)bt_tx_msg, BT_TX_MSG_SIZE, "%s%.5u", LCcommand[REQ_DACRC], (int)*((int *)data));
	if (bt_tx_msg_len > (BT_PKT_CMD_SIZE + BT_PKT_DATA_SIZE)) {
		//h ERR("DARC adc_data data overflow!!\n");
	}
	else {
		bt_send_packet((struct bt_message *)bt_tx_msg);
	}
	return 0;
}

int func_cmd_DAST(unsigned char *data)
{
	uint16_t dac1;
	uint16_t dac2;
	int set_value = strtoint_n(data, BT_PKT_DATA_SIZE);
	set_value = set_value / 1000;
	dac1 = ((voltage_value[set_value] - 32768) & 0xff00) >> 8;
	dac2 = (voltage_value[set_value] - 32768) & 0xff;
	uint16_t dac3[3] = {0x00, dac1, dac2};
	//set dac
//	dac_datas(dac3);
    spi1_dac_cs0_init();
	uint8_t value[6];
	uint8_t rcv_value[6];
	for(char i = 0; i < 3; i++)
	{
		value[i*2] = dac3[i];
		value[(i*2)+1] = dac3[i] >> 8;
	}	
 	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi1_dac, value, 6, rcv_value, 7));
	
	//response dac
	bt_proc_response_packet(REQ_DACSETRC, (unsigned char *)data);
	nv_dac_voltage = strtoint_n(data, BT_PKT_DATA_SIZE);
	return 0;		//h 추가
}

int func_cmd_DASR(unsigned char *data)
{
	int bt_tx_msg_len;
	memset(bt_tx_msg, 0, sizeof(bt_tx_msg));
	bt_tx_msg_len = snprintf((char *)bt_tx_msg, BT_TX_MSG_SIZE, "%s%.5u", LCcommand[REQ_DACSETRC], strtoint_n(data, BT_PKT_DATA_SIZE));
	if (bt_tx_msg_len > (BT_PKT_CMD_SIZE + BT_PKT_DATA_SIZE)) {
		//h ERR("DASR adc_data data overflow!!\n");
	}
	else {
		bt_send_packet((struct bt_message *)bt_tx_msg);
	}
	return 0;
}

int func_cmd_ADSE(unsigned char *data)
{
	int set_value = strtoint_n(data, BT_PKT_DATA_SIZE);
//h 	adc_start_timer();
	if (nv_adc_mode > 2) {
		uart1_printf("ADSE packet error(data)\r\n");
		rx1_enter = 0;
	}
	nv_adc_mode = set_value;
	return 0;
}

int func_cmd_ADmS(unsigned char *data)
{
	double l1_data = *((unsigned long *)data);
	double l2_data;
	int bt_tx_msg_len;
	int adc_data;
	l2_data = ((5 * (l1_data - 0x813500) / 0x7eaf00)) + 0.051 - 0.0009;
	adc_data = (int)(l2_data * 10000);
	if (adc_data < 0) adc_data = 0;
	else if (adc_data > 50000) adc_data = 50000;
	memset(bt_tx_msg, 0, sizeof(bt_tx_msg));
	bt_tx_msg_len = snprintf((char *)bt_tx_msg, BT_TX_MSG_SIZE, "%s%.5u", LCcommand[REQ_ADmST], adc_data);
	if (bt_tx_msg_len > (BT_PKT_CMD_SIZE + BT_PKT_DATA_SIZE)) {
		//h ERR("ADmS adc_data data overflow!!\n");
	}
	else {
		bt_send_packet((struct bt_message *)bt_tx_msg);
	}
	return 0;
}

int func_cmd_ADnS(unsigned char *data)
{
	double l1_data = *((unsigned long *)data);
	double l2_data;
	int bt_tx_msg_len;
	int adc_data = 0;
	l2_data = ((5 * (l1_data - 8404000)) / 8365900) + 0.012;
	adc_data = (int)(l2_data * 10000);
	if (adc_data < 0) adc_data = 0;
	else if (adc_data > 50000) adc_data = 50000;
	memset(bt_tx_msg, 0, sizeof(bt_tx_msg));
	bt_tx_msg_len = snprintf((char *)bt_tx_msg, BT_TX_MSG_SIZE, "%s%05d", LCcommand[REQ_ADnST], adc_data);
	if (bt_tx_msg_len > (BT_PKT_CMD_SIZE + BT_PKT_DATA_SIZE)) {
		//h ERR("ADnS adc_data data overflow!!\n");
	}
	else {
		bt_send_packet((struct bt_message *)bt_tx_msg);
	}
	return 0;
}

int func_cmd_ADKS(unsigned char *data)
{
	unsigned long l1_data = *((unsigned long *)data);
	unsigned long l2_data;
	int bt_tx_msg_len;
	int adc_data;
	l2_data = (int)(((5 * (l1_data - 0x813500)) / 0x7eaf00) + 0.050);
	adc_data = (int)(l2_data * 10000);
	if (adc_data < 0) adc_data = 0;
	else if (adc_data > 50000) adc_data = 50000;
	memset(bt_tx_msg, 0, sizeof(bt_tx_msg));
	bt_tx_msg_len = snprintf((char *)bt_tx_msg, BT_TX_MSG_SIZE, "%s%.5u", LCcommand[REQ_ADKST], adc_data);
	if (bt_tx_msg_len > (BT_PKT_CMD_SIZE + BT_PKT_DATA_SIZE)) {
		//h ERR("ADKS adc_data data overflow!!\n");
	}
	else {
		bt_send_packet((struct bt_message *)bt_tx_msg);
	}
	return 0;
}

void bt_proc_response_packet(   LCcmd_e_type cmd, unsigned char *data)
{
	if (cmd < REQ_CMDMAX) {
		func_arry[cmd].func(data);
	}
	else {
		//h ERR("bt send none cmd = %d\n", cmd);
	}
}

void bt_proc_request_packet(struct bt_message msg)
{
	int i = 0;
	while (i < REQ_CMDMAX) {
		if (strncmp(LCcommand[i], (unsigned char *)&msg.cmd, 4) == 0) {
			if (func_arry[i].func != NULL) func_arry[i].func((unsigned char *)&msg.data);
			break;
		}
		i++;
	}
	if(i == REQ_CMDMAX) {
		//h ERR("bt receive none cmd = %d\n", i);
	}
}

