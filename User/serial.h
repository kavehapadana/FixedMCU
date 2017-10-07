#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "lpc17xx_libcfg.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "LPC17xx.h"
#include "lpc_types.h"
#define USART_BUFFER_SIZE 256

void init_usart0(void);
void init_usart1(void);
void init_usart2(void);
void init_usart3(void);


class SerialPort
{
public:
	SerialPort(unsigned char);
	//~SerialPort();
	void dummy();
	int InitPort(unsigned long Baudrate,int Priority);

	char getch();
	void putch(char Data);
	void puts(char *s);
	void ProcPort(void);
	void OutProcPort(void);
  void UART_ConfigStruct(UART_CFG_Type *UART_InitStruct, uint32_t baudrate);
	unsigned char * gets(unsigned char * s);
	void write(unsigned char *buf, unsigned len);
	unsigned DataAvail();
	unsigned LineStatus();
	void reset();
	void ClearPendingInterrupts();
	//unsigned ComBase;
	LPC_UART_TypeDef *ComBase;
//
	unsigned oTail, oHead, iTail, iHead;
	char iBuff[USART_BUFFER_SIZE], oBuff[USART_BUFFER_SIZE];
private:
	int ActivePort;
};

// void RxProc();
// void TxProc();

extern SerialPort COM_Port;
#endif
