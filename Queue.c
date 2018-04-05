/* ======================================
			Pa-Fi Project
			Circular Queue for Uart Rx
========================================*/

#include "Queue.h"

uint8_t	U1_QUEUE[MAX_QUEUE];
uint8_t	U2_QUEUE[MAX_QUEUE];
uint8_t	U3_QUEUE[MAX_QUEUE];

unsigned int front1, rear1;
unsigned int front2, rear2;
unsigned int front3, rear3;

void InitQueue1()
{
	front1 =  rear1 = 0;
}

//h void ClearQueue1()
//h {
//h   USART_Cmd(USART1, DISABLE);
//h 	front1 =  rear1;
//h   USART_Cmd(USART1, ENABLE);
//h }

//h uint8_t PutByte1(uint8_t n)
//h {
//h 	//If queue is full
//h 	if((rear1 +1) % MAX_QUEUE == front1)
//h 	{
//h 		uart1_printf("Queue1 overflow\r\n");
//h 		return 0xff;
//h 	}
//h   USART_Cmd(USART1, DISABLE);
//h 	U1_QUEUE[rear1] = n;
//h 	rear1++;
//h 	rear1 %= MAX_QUEUE;
//h   USART_Cmd(USART1, ENABLE);
//h 
//h 	if(rear1 >= MAX_QUEUE)
//h 	{
//h 		uart1_printf("Queue1 overflow-!\r\n");
//h 		return 0xff;
//h 	}
//h 
//h 	return n;
//h }


uint8_t GetByte1()
{
	uint8_t n;

	//If queue is empty
	if(front1 == rear1)
	{
		//uart1_printf("The Queue1 is empty\r\n");
		return 0xff;
	}

  //USART_Cmd(USART1, DISABLE);
	n = U1_QUEUE[front1];
	front1++;
	front1 %= MAX_QUEUE;
  //USART_Cmd(USART1, ENABLE);

	if(front1 >= MAX_QUEUE)
	{
		uart1_printf("Queue1 overflow-!\r\n");
		return 0xff;
	}

	return n;
}

void PrintQueue1()
{
	unsigned int i;

  //USART_Cmd(USART1, DISABLE);
	for(i=front1; i!=rear1; i = ((i+1) % MAX_QUEUE))
		uart1_printf("%02x", U1_QUEUE[i]);
  //USART_Cmd(USART1, ENABLE);

	uart1_printf("\r\n");

}

void InitQueue2()
{
	front2 =  rear2 = 0;
}

//h void ClearQueue2()
//h {
//h   USART_Cmd(USART2, DISABLE);
//h 	front2 =  rear2;
//h   USART_Cmd(USART2, ENABLE);
//h }

//h uint8_t PutByte2(uint8_t n)
//h {
//h 	//If queue is full
//h 	if((rear2 +1) % MAX_QUEUE == front2)
//h 	{
//h 		uart1_printf("Queue2 overflow\r\n");
//h 		return 0xff;
//h 	}
//h   USART_Cmd(USART2, DISABLE);
//h 	U2_QUEUE[rear2] = n;
//h 	rear2++;
//h 	rear2 %= MAX_QUEUE;
//h   USART_Cmd(USART2, ENABLE);
//h 
//h 	if(rear2 >= MAX_QUEUE)
//h 	{
//h 		uart1_printf("Queue2 overflow-!\r\n");
//h 		return 0xff;
//h 	}
//h 	return n;
//h }
//h 
//h uint8_t GetByte2()
//h {
//h 	uint8_t n;
//h 
//h 	//If queue is empty
//h 	if(front2 == rear2)
//h 	{
//h 		//uart1_printf("The Queue2 is empty\r\n");
//h 		return 0xff;
//h 	}
//h 
//h   //USART_Cmd(USART2, DISABLE);
//h 	n = U2_QUEUE[front2];
//h 	front2++;
//h 	front2 %= MAX_QUEUE;
//h   //USART_Cmd(USART2, ENABLE);
//h 
//h 	if(front2 >= MAX_QUEUE)
//h 	{
//h 		uart1_printf("Queue2 overflow-!\r\n");
//h 		return 0xff;
//h 	}
//h 	return n;
//h }

//h void PrintQueue2()
//h {
//h 	unsigned int i;
//h 
//h   //USART_Cmd(USART2, DISABLE);
//h 	for(i=front2; i!=rear2; i = ((i+1) % MAX_QUEUE))
//h 		uart1_printf("%02x", U2_QUEUE[i]);
//h 
//h 	uart1_printf("\r\n");
//h   //USART_Cmd(USART2, ENABLE);
//h 
//h }

//h void InitQueue3()
//h {
//h 	front3 =  rear3 = 0;
//h }
//h 
//h void ClearQueue3()
//h {
//h   USART_Cmd(USART3, DISABLE);
//h 	front3 =  rear3;
//h   USART_Cmd(USART3, ENABLE);
//h }

//h uint8_t PutByte3(uint8_t n)
//h {
//h 	//If queue is full
//h 	if((rear3 +1) % MAX_QUEUE == front3)
//h 	{
//h 		uart1_printf("Queue3 overflow\r\n");
//h 		return 0xff;
//h 	}
//h   USART_Cmd(USART3, DISABLE);
//h 	U3_QUEUE[rear3] = n;
//h 	rear3++;
//h 	rear3 %= MAX_QUEUE;
//h   USART_Cmd(USART3, ENABLE);
//h 
//h 	if(rear3 >= MAX_QUEUE)
//h 	{
//h 		uart1_printf("Queue3 overflow-!\r\n");
//h 		return 0xff;
//h 	}
//h 	return n;
//h }


//h uint8_t GetByte3()
//h {
//h 	uint8_t n;
//h 
//h 	//If queue is empty
//h 	if(front3 == rear3)
//h 	{
//h 		//uart1_printf("The Queue3 is empty\r\n");
//h 		return 0xff;
//h 	}
//h 
//h   //USART_Cmd(USART3, DISABLE);
//h 	n = U3_QUEUE[front3];
//h 	front3++;
//h 	front3 %= MAX_QUEUE;
//h   //USART_Cmd(USART3, ENABLE);
//h 
//h 	if(front3 >= MAX_QUEUE)
//h 	{
//h 		uart1_printf("Queue3 overflow-!\r\n");
//h 		return 0xff;
//h 	}
//h 	return n;
//h }

//h void PrintQueue3()
//h {
//h 	unsigned int i;
//h 
//h   //USART_Cmd(USART3, DISABLE);
//h 	for(i=front3; i!=rear3; i = ((i+1) % MAX_QUEUE))
//h 		uart1_printf("%02x", U3_QUEUE[i]);
//h   //USART_Cmd(USART3, ENABLE);
//h 
//h 	uart1_printf("\r\n");
//h 
//h }

/* Queue API for BT */
int GetQueue2(uint8_t *data)
{
	uint8_t n;
	if(front2 == rear2)	{
		return FALSE;
	}

//h 	INT_LOCK();
	n = U2_QUEUE[front2];
	front2++;
	front2 %= MAX_QUEUE;
//h 	INT_FREE();
	*data = n;
	return TRUE;
}

int CheckQueue2()
{
	return (front2 == rear2) ? FALSE : TRUE;
}

uint16_t ReadString2(uint8_t* buf)
{
	int len = 0;
	uint8_t rx;
	while(1) {
		if (!GetQueue2(&rx)) continue;
		if (rx == '\n') {
			buf[len++] = '\0';
			break;
		} else {
			buf[len++] = rx;
		}
	}
	return len;
}
