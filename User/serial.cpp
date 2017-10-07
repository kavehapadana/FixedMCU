#include "serial.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdio.h>
extern SerialPort GYRO_Port;
	  PINSEL_CFG_Type PinCfg;

void init_usart0(void){
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 2;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 3;
    PINSEL_ConfigPin(&PinCfg);
}
void init_usart1(void){
	
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 16;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 15;
    PINSEL_ConfigPin(&PinCfg);
}

void init_usart2(void){
	
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);
}
void init_usart3(void){
	
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}
//

/*************************************************************************/
/*************************************************************************/
unsigned char * SerialPort::gets(unsigned char * s)
{
	unsigned char ch=0,*i=s;
	do{
		*i++=ch=getch();
		*i=0;
		}while(ch>13);
	return s;
}
/*************************************************************************/
unsigned SerialPort::DataAvail()
{
	return ((iHead-iTail));
}
/*************************************************************************/
char SerialPort::getch()
{	
	while(!DataAvail());
	char c = iBuff[(iTail++)];
	if (iTail>=USART_BUFFER_SIZE)
		iTail=0;
	//iTail%=USART_BUFFER_SIZE;
	return c;
}
/*************************************************************************/
void SerialPort::reset()
{
	iHead=iTail=oHead=oTail=0;
}
/************************************************************************/
void SerialPort::putch(char data)
{
	oBuff[(oTail++)]=data;
	if (oTail >= USART_BUFFER_SIZE)
		oTail = 0; //USART2_Tail - USART_BUFFER_SIZE;

//	U2_Out_Buf[U2_oTail++] = data;	
	
//	if (U2_oTail >= USART_BUFFER_SIZE)
//		U2_oTail = 0; //USART2_Tail - USART_BUFFER_SIZE;
	UART_IntConfig(ComBase, UART_INTCFG_THRE, ENABLE); // enable the USART2 transmit interrupt
}
/****************************************************************************/

/************************************************************************/
void SerialPort::puts(char *s)
{
	int i=0;
	while (s[i])  putch(s[i++]);
}
/*************************************************************************/
void SerialPort::ClearPendingInterrupts()
{

}
/********************************************************************/
int SerialPort::InitPort(unsigned long Baudrate,int Priority)
{
	// UART Configuration structure variable
  UART_CFG_Type UARTConfigStruct;
  // UART FIFO configuration Struct variable
  UART_FIFO_CFG_Type UARTFIFOConfigStruct;
  // Pin configuration
  PINSEL_CFG_Type PinCfg;
  
  /* Initialize UART Configuration parameter structure:
   * Baudrate = baudrate bps
   * 8 data bit
   * 1 Stop bit
   * None parity
   */
  
  UART_ConfigStruct(&UARTConfigStruct,Baudrate);
  
  /* Initialize FIFOConfigStruct to default state:
   * 				- FIFO_DMAMode = DISABLE
   * 				- FIFO_Level = UART_FIFO_TRGLEV0
   * 				- FIFO_ResetRxBuf = ENABLE
   * 				- FIFO_ResetTxBuf = ENABLE
   * 				- FIFO_State = ENABLE
   */
  UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	
	
	LPC_UART_TypeDef *UARTx;
	switch (ActivePort) {
		case 0:
			init_usart0();
			UARTx = LPC_UART0;
			break;
		case 1:
			init_usart1();
			UARTx = (LPC_UART_TypeDef*)LPC_UART1;
		break;
		case 2:
			init_usart2();
			UARTx = LPC_UART2;
		break;
		case 3:
			init_usart3();
			UARTx = LPC_UART3;
		break;
	};
	
	    // Initialize UART0 peripheral with given to corresponding parameter
    UART_Init((LPC_UART_TypeDef *)UARTx, &UARTConfigStruct);
    // Initialize FIFO for UART0 peripheral
    UART_FIFOConfig((LPC_UART_TypeDef *)UARTx, &UARTFIFOConfigStruct);
    // Enable UART Transmit
    UART_TxCmd((LPC_UART_TypeDef *)UARTx, DISABLE);

    /* Enable UART Rx interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)UARTx, UART_INTCFG_RBR, ENABLE);
    /* Enable UART line status interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)UARTx, UART_INTCFG_RLS, ENABLE);
    /*
     * Do not enable transmit interrupt here, since it is handled by
     * UART_Send() function, just to reset Tx Interrupt state for the
     * first time
     */


	switch (ActivePort) 
		{
			case 0:
				/* preemption = 1, sub-priority = 1 */
				NVIC_SetPriority(UART0_IRQn, Priority);
						/* Enable Interrupt for UART0 channel */
				NVIC_EnableIRQ(UART0_IRQn);
			break;
			case 1:
				/* preemption = 1, sub-priority = 1 */
				NVIC_SetPriority(UART0_IRQn, Priority);
						/* Enable Interrupt for UART0 channel */
				NVIC_EnableIRQ(UART0_IRQn);
			break;
			case 2:
				/* preemption = 1, sub-priority = 1 */
				NVIC_SetPriority(UART0_IRQn, Priority);
						/* Enable Interrupt for UART0 channel */
				NVIC_EnableIRQ(UART0_IRQn);
			break;
			case 3:
				/* preemption = 1, sub-priority = 1 */
				NVIC_SetPriority(UART0_IRQn, Priority);
						/* Enable Interrupt for UART0 channel */
				NVIC_EnableIRQ(UART0_IRQn);
			break;
	}	
	return true;
}

inline unsigned SerialPort::LineStatus()
{
	return 0;
// 	return inp(ComBase+LSR);
}
/*************************************************************************/
/*                                                                       */
/*************************************************************************/
void SerialPort::UART_ConfigStruct(UART_CFG_Type *UART_InitStruct, uint32_t baudrate)
{
  UART_InitStruct->Baud_rate = baudrate;
  UART_InitStruct->Databits = UART_DATABIT_8;
  UART_InitStruct->Parity = UART_PARITY_NONE;
  UART_InitStruct->Stopbits = UART_STOPBIT_1;
}


void SerialPort::write(unsigned char *buf, unsigned len)
{
	for (int i=0;i<len;i++)
		putch(*(buf+i));
}
/*
SerialPort::~SerialPort()
{
	ClearPendingInterrupts(); // Clean up
}
*/

SerialPort::SerialPort(uint8_t ComPort)
{

	ActivePort = ComPort;
	switch (ComPort) 
		{
		case 0:
			ComBase = (LPC_UART_TypeDef *)LPC_UART0;
			break;
		case 1:
			ComBase = (LPC_UART_TypeDef *)LPC_UART1;
			break;
		case 2:
			ComBase = (LPC_UART_TypeDef *)LPC_UART2;
			break;
		case 3:
			ComBase = (LPC_UART_TypeDef *)LPC_UART3;
			break;
		}	
}

void SerialPort::dummy()
{
}

