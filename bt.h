#ifndef __BT_H
#define __BT_H

#include <ctype.h>
#include "Queue.h"

#define BT_PKT_CMD_SIZE			4
#define BT_PKT_DATA_SIZE		5

enum bt_mldp_state {
	BT_MLDP_NOT_CONNECTED,
	BT_MLDP_CONNECTED
};

struct bt_message
{
    unsigned char cmd[BT_PKT_CMD_SIZE];
    unsigned char data[BT_PKT_DATA_SIZE];
};

bool bt_receive_packet(struct bt_message *message);
bool bt_send_packet(struct bt_message *message);
int bt_setup_module(char *version);
bool bt_check_response(char *data);
bool bt_reboot(void);

extern int CheckQueue2();
extern void Serial2_PutString(char *s);
extern void SerialPutChar2(char c);
#endif
