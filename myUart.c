#include "myUart.h"

#define UART_BUFFER_LENGTH 128
unsigned char uart_cmdBuf[UART_BUFFER_LENGTH] = {0};
uint32_t uart_buffer_count = 0;

extern void UART_EnableInt(UART_T*, uint32_t);
extern void UART_DisableInt(UART_T*, uint32_t);

//Max length is 128 chars
void myUartDebugSend(const char* str)
{
	uint8_t buf[128] = {0};
	int bufLen = strlen(str);
	memcpy(buf, str, bufLen);
	myUartSendDataByte(buf, bufLen);
}

void myUartOpen()
{
	UART_EnableInt(UART_NUMBER, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

void myUartClose()
{
	UART_DisableInt(UART_NUMBER, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

void uartHandle()  
{
	uint8_t u8InChar = 0xFF;
	uint32_t u32IntSts = UART_NUMBER->INTSTS;

	if(u32IntSts & UART_INTSTS_RDAINT_Msk)
	{
		/* Get all the input characters */
		while(UART_IS_RX_READY(UART_NUMBER)) 
		{
			/* Get the character from UART Buffer */
			u8InChar = UART_READ(UART_NUMBER);   
			uart_cmdBuf[uart_buffer_count++] = u8InChar;
		}
	}
}

void myUartSendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{

	uint32_t writeCnt = 0;
	writeCnt = UART_Write(UART_NUMBER, pu8TxBuf, u32WriteBytes);
}




int myUartRead(char *data)
{
	if (uart_buffer_count == 0) return 0;
	
	memcpy(data, uart_cmdBuf, uart_buffer_count);
	
	int count = uart_buffer_count;
	uart_buffer_count = 0;
	
	return count;
}
