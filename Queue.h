/* ======================================
			Pa-Fi Project
			Circular Queue for Uart Rx
========================================*/

#include "main.h"

#define MAX_QUEUE	512


extern uint8_t	U1_QUEUE[MAX_QUEUE];
extern uint8_t	U2_QUEUE[MAX_QUEUE];
extern uint8_t	U3_QUEUE[MAX_QUEUE];

extern unsigned int front1, rear1;
extern unsigned int front2, rear2;
extern unsigned int front3, rear3;

extern void InitQueue1();

extern void ClearQueue1();

extern uint8_t PutByte1(uint8_t n);

extern uint8_t GetByte1();

extern void PrintQueue1();


extern void InitQueue2();

extern void ClearQueue2();

extern void ClearQueue2p();

//h extern uint8_t PutByte2(uint8_t n);

//h extern uint8_t GetByte2();

//h extern void PrintQueue2();


//h extern void InitQueue3();

//h extern void ClearQueue3();

//h extern uint8_t PutByte3(uint8_t n);

//h extern uint8_t GetByte3();

//h extern void PrintQueue3();

int GetQueue2(uint8_t *data);
int CheckQueue2(void);

