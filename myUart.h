#include "M451Series.h"

#define USE_UART_NUMBER 0
#if (USE_UART_NUMBER==0)
#define UART_NUMBER UART0
#endif

#if (USE_UART_NUMBER==1)
#define UART_NUMBER UART1
#endif

#define UART_READ_TIMEOUT 2	//timeout = UART_READ_TIMEOUT * 200ms
#define UART_BUFFER_LENGTH 128

#define UART_CMD_NONE 0x00
#define UART_CMD_SET_MUX 0x11

#define UART_CMD_SET_START_FREQ 0x40
#define UART_CMD_SET_FREQ_INCR 0x41
#define UART_CMD_SET_NUM_INCR  0x42
#define UART_CMD_SET_GAIN  0x43
#define UART_CMD_START_SWEEP 0x44
#define UART_CMD_RECALIBRATION 0x45
#define UART_CMD_SET_REF_RESIST 0x46
#define UART_CMD_GET_IMPEDANCE_PART 0x47
#define UART_CMD_GET_PHASE_PART 0x48
#define UART_CMD_AD5933_RELEASE 0x49
#define UART_CMD_CALIBRATION_COMPLETED 0x50



//TODO debug
extern unsigned int strlen(const char*);
extern void* memcpy(void*, const void*, unsigned int);
//end debug

void myUartOpen(void);
void myUartClose(void);
void UART0_IRQHandler(void);

void myUartSendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void myUartSendByte(uint8_t pu8TxBuf);
void myUartDebugSend(const char* str);
int myUartRead(char *data);
