
#include "bt.h"

#define BT_RX_BUFFER_SIZE 		50

#define BT_FEATURE_STR			"36000800"
#define BT_VENDOR_NAME_STR		"BT CNR"
//#define BT_PKT_SOF				'A'
//#define BT_PKT_EOF				'B'
#define BT_PKT_LINEFEED		'\n'

enum bt_pkt_rx_state {
	BT_PKT_RX_SOF_S,
	BT_PKT_RX_CMD_S,
	BT_PKT_RX_CMD_DATA_S,
	BT_PKT_RX_EOF_S,
	BT_PKT_SOF = 'A',
	BT_PKT_EOF = 'B'
};

#define BT_RX_BUFFER_SIZE    50

static uint8_t bt_rx_buffer[BT_RX_BUFFER_SIZE];

int bt_check_rx_buffer(void)
{
	return CheckQueue2();
}

int bt_read_rx_buffer(uint8_t *data)
{
	return GetQueue2(data);
}

void bt_flush_rx_buffer(void)
{
	uint8_t data;
	while(bt_check_rx_buffer()) {
		bt_read_rx_buffer(&data);
	}
}

void bt_send_command(char *s)
{
//bt 모듈로 uart를 통하여 명령을 전송하는 것으로 보임
//		Serial2_PutString(s);
}

void bt_send_byte(char c)
{
//bt 모듈로 uart를 통하여 명령을 전송하는 것으로 보임		
//	SerialPutChar2(c);
}

void bt_handle_timeout()
{
}

void bt_enable_flow_control(void)
{
//h 	USART2_EnableFlowCtrl();
}

uint8_t bt_start_timer(unsigned int timer_id, unsigned long time )
{
#if 0
	extern BYTE timer_start(unsigned int timn, unsigned long tvalue, FuncPtr func);
	return timer_start(timer_id, time, bt_handle_timeout);
#else
	return 1;
#endif
}

uint8_t bt_has_timer_expired(unsigned int timer_id)
{
	uint8_t timer_use(unsigned int timn);
	return timer_use(timer_id) ? FALSE : TRUE;
}

/*
uint8_t bt_stop_timer(unsigned int timer_id)
{
	return 1;
	extern BYTE timer_start(unsigned int timn, unsigned long tvalue, FuncPtr func);
	return timer_start(timer_id, time, bt_handle_timeout);
}
*/

bool bt_send_packet(struct bt_message *message)
{
	int i;
	bt_send_byte(BT_PKT_SOF);
	for (i = 0 ; i < BT_PKT_CMD_SIZE; i++)
		bt_send_byte(message->cmd[i]);
	for (i = 0 ; i < BT_PKT_DATA_SIZE; i++)
		bt_send_byte(message->data[i]);
	bt_send_byte(BT_PKT_EOF);
	bt_send_byte('\n');
	return 0;
}

bool bt_receive_packet(struct bt_message *message)
{
	static enum bt_pkt_rx_state state = BT_PKT_RX_SOF_S;
	unsigned char ch;
	int cmd_count = 0;
	int data_count = 0;
	while (bt_check_rx_buffer()) {
		bt_read_rx_buffer(&ch);
//h 		LOG("RX data=0x%x(%c)\n", ch, ch);
		switch(state) {
		case BT_PKT_RX_SOF_S:
			if(ch == BT_PKT_SOF)
				state = BT_PKT_RX_CMD_S;
			else {
				//state = BT_PKT_SOF;
				//h ERR("BT_PKT_RX_SOF_S failed: not SOF data=0x%x(%c)\n", ch, ch);
			}
			break;
		case BT_PKT_RX_CMD_S:
			if (isalpha((int)ch)) {
				message->cmd[cmd_count] = ch;
				cmd_count++;
				if (cmd_count == BT_PKT_CMD_SIZE) {
					state = BT_PKT_RX_CMD_DATA_S;
				}
			}
			else {
				state = BT_PKT_SOF;
				//h ERR("BT_PKT_RX_CMD_S failed: !is_alpha data=0x%x(%c)\n", ch, ch);
			}
			break;
		case BT_PKT_RX_CMD_DATA_S:
			if (isdigit(ch)) {
				message->data[data_count] = ch;
				data_count++;
				if (data_count == BT_PKT_DATA_SIZE) {
					state = BT_PKT_RX_EOF_S;
				}
			}
			else {
				//h ERR("BT_PKT_RX_CMD_DATA_S failed: !is_digit data=0x%x(%c)\n", ch, ch);
				state = BT_PKT_SOF;
			}
			break;
		case BT_PKT_RX_EOF_S:
			if(ch == BT_PKT_LINEFEED /*BT_PKT_EOF*/) {
  			state = BT_PKT_RX_SOF_S;
				return TRUE;
			}
			else {
				////h ERR("BT_PKT_RX_CMD_DATA_S EOF data=0x%x(%c)\n", ch, ch);
			}
			break;
		default:
			state = BT_PKT_RX_SOF_S;
		}

		if(ch == BT_PKT_LINEFEED) {
			state = BT_PKT_RX_SOF_S;
		}
    
	}

  if (state == BT_PKT_SOF)  state = BT_PKT_RX_SOF_S;

	return FALSE;
}

int bt_get_response(char *data)
{
	unsigned int byte_count = 0;
	unsigned char new_byte;
//h		//h bt_start_timer(TIMER_BT_ID, 1000);
	while(byte_count < BT_RX_BUFFER_SIZE) {
		if(bt_check_rx_buffer()) {
			bt_read_rx_buffer(&new_byte);
//h			LOG("%c", new_byte);
			*data++ = new_byte;
			byte_count++;
			if(new_byte == '\n') {
				return TRUE;
			}
		}
//h 		if(bt_has_timer_expired(TIMER_BT_ID)) {
			//h ERR("bt_get_response timer expired\n");
//h 			return FALSE;
//h 		}
	}
	return FALSE;
}

int bt_compare_response(char *data1, char *data2)
{
	unsigned int i;
	for(i = 0; i < BT_RX_BUFFER_SIZE; i++) {
		if(*data1 == '\0')
			return TRUE;
		else if(*data1++ != *data2++)
			return FALSE;
	}
	return FALSE;
}

bool bt_check_response(char *data)
{
	unsigned int byte_count = 0;
	unsigned char new_byte, *buf_ptr;
//h 	//h bt_start_timer(TIMER_BT_ID, 1000);
	buf_ptr = bt_rx_buffer;
	while(byte_count < BT_RX_BUFFER_SIZE) {
		if(bt_check_rx_buffer()) {
			bt_read_rx_buffer(&new_byte);
//h			LOG("%c", new_byte);
			*buf_ptr++ = new_byte;
			byte_count++;
			if(new_byte == '\n') {
				break;
			}
		}
//h 		if(bt_has_timer_expired(TIMER_BT_ID))
//h 			return FALSE;
	}
	return !memcmp(data, bt_rx_buffer, byte_count) ? TRUE : FALSE;
}

int bt_check_response_with_wildcard(char *data, char wildcard)
{
	unsigned int byte_count = 0;
	unsigned char new_byte, *buf_ptr;
	//h bt_start_timer(TIMER_BT_ID, 200);
	buf_ptr = bt_rx_buffer;
	while(byte_count < BT_RX_BUFFER_SIZE) {
		if(bt_check_rx_buffer()) {
			bt_read_rx_buffer(&new_byte);
			*buf_ptr++ = new_byte;
			byte_count++;
			if(new_byte == '\n')
				break;
		}
//h 		if(bt_has_timer_expired(TIMER_BT_ID))
//h 			return FALSE;
	}
	return !memcmp(data, bt_rx_buffer, byte_count) ? TRUE : FALSE;
}

int bt_setup_module(char *version)
{
	unsigned int i, changes = FALSE;
	nrf_delay_ms(100);
	//bt_temp();
	bt_send_command("v\r");
	if (!bt_get_response((char *)bt_rx_buffer)) {
		return 2;
	}
	for (i = 0; i < 8; i++) {
		version[i] = bt_rx_buffer[i + 11];
		if(version[i] == ' ') {
			version[i] = '\0';
			break;
		}
	}
	version[8] = '\0';
	bt_send_command("gr\r");
	if (!bt_check_response(BT_FEATURE_STR "\r\n")) {
		bt_send_command("sr," BT_FEATURE_STR "\r");
		if(!bt_check_response("AOK\r\n"))
			return 3;
		changes = TRUE;
	}
	bt_send_command("gn\r");
	if (!bt_check_response(BT_VENDOR_NAME_STR "\r\n")) {
		bt_send_command("sn," BT_VENDOR_NAME_STR "\r");
		if(!bt_check_response("AOK\r\n"))
			return 4;
		changes = TRUE;
	}
	if (changes == TRUE) {
		bt_send_command("wc\r");
		if(!bt_check_response("AOK\r\n"))
			return 5;
		if(!bt_reboot())
			return 6;
		bt_send_command("gr\r");
		if(!bt_check_response(BT_FEATURE_STR "\r\n"))
			return 7;
		bt_send_command("gn\r");
		if(!bt_check_response(BT_FEATURE_STR "\r\n"))
			return 8;
	}
	bt_enable_flow_control();
//h	 LOG("bt_setup_module done\n");
	return 0;
}

bool bt_reboot(void)
{
	unsigned char ch;
	bt_send_command("r,1\r");
	if(!bt_check_response("Reboot\r\n"))
		return FALSE;
	//h bt_start_timer(TIMER_BT_ID, 200);
	while(bt_check_rx_buffer()) {
//h 		if(bt_has_timer_expired(TIMER_BT_ID))
//h 			return FALSE;
	}
	if (bt_read_rx_buffer(&ch) != 0)
		return FALSE;
	//h bt_start_timer(TIMER_BT_ID, 3000);
	while(bt_check_rx_buffer()) {
//h 		if(bt_has_timer_expired(TIMER_BT_ID))
//h 			return FALSE;
	}
	if(!bt_check_response("CMD\r\n"))
		return FALSE;
	return TRUE;
}
