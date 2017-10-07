/***********************************************************************//**
 * @file		User_Uart.h
 * @brief		Contains all macro definitions and function prototypes
 * 				support for UART firmware library on LPC17xx
 * @version		1.00
 * @date		1. January. 2017
 * @author		ElecSys Co.
 **************************************************************************
 * Software that is described herein is Peripheral Layer2 Functions for UART
 **************************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup LAYER2 UART
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef __USER_UART_H
#define __USER_UART_H

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_uart.h"
#include "lpc17xx_libcfg.h"
#include "lpc17xx_pinsel.h"
#include "LPC17xx.h"
#include "lpc_types.h"

/* Public Macros -------------------------------------------------------------- */
/** @defgroup LAYER2_UART_Public_Macros
 * @{
 */

/* buffer size definition */
#define UART_RING_BUFSIZE 256

/* Buf mask */
#define __BUF_MASK (UART_RING_BUFSIZE-1)
/* Check buf is full or not */
#define __BUF_IS_FULL(head, tail) ((tail&__BUF_MASK)==((head+1)&__BUF_MASK))
/* Check buf will be full in next receiving or not */
#define __BUF_WILL_FULL(head, tail) ((tail&__BUF_MASK)==((head+2)&__BUF_MASK))
/* Check buf is empty */
#define __BUF_IS_EMPTY(head, tail) ((head&__BUF_MASK)==(tail&__BUF_MASK))
/* Reset buf */
#define __BUF_RESET(bufidx)	(bufidx=0)
#define __BUF_INCR(bufidx)	(bufidx=(bufidx+1)&__BUF_MASK)

#define UART_0 0
#define UART_1 1
#define UART_2 2
#define UART_3 3

#define UART0 LPC_UART0
#define UART1 (LPC_UART_TypeDef *)LPC_UART1
#define UART2 LPC_UART2
#define UART3 LPC_UART3

/**
 * @}
 */


/* Public Types --------------------------------------------------------------- */
/** @defgroup LAYER2_UART_Public_Types UART
 * @{
 */

/** @brief UART Ring buffer structure */
typedef struct
{
    __IO uint8_t RxBufferRingEn;          /*!< UART Rx ring buffer Enable Status */
    __IO uint32_t tx_head;                /*!< UART Tx ring buffer head index */
    __IO uint32_t tx_tail;                /*!< UART Tx ring buffer tail index */
    __IO uint32_t rx_head;                /*!< UART Rx ring buffer head index */
    __IO uint32_t rx_tail;                /*!< UART Rx ring buffer tail index */
    __IO uint8_t  tx[UART_RING_BUFSIZE];  /*!< UART Tx data ring buffer */
    __IO uint8_t  rx[UART_RING_BUFSIZE];  /*!< UART Rx data ring buffer */
} UART_RING_BUFFER_T;

/**
 * @}
 */

/* Public Functions ----------------------------------------------------------- */
/** @defgroup LAYER 2_UART_Public_Functions UART
 * @{
 */
/* Interrupt service routines */
void UART0_IRQHandler(void);
extern void UART2_IRQHandler(void);
void UART_IntErr(uint8_t bLSErrType);
void UART_IntTransmit(LPC_UART_TypeDef *UARTx);
void UART_IntReceive(LPC_UART_TypeDef *UARTx);
void Send_uint_AC(uint8_t* data,int len);
void Send_uint_AC_rb(uint8_t* data,int len);


void Com_Init(LPC_UART_TypeDef *UARTx,FunctionalState transmitEn, uint8_t interruptEn, uint32_t Priority, uint32_t baudrate, uint8_t RxBufferRingEn);
void UART_ConfigStruct(UART_CFG_Type *UART_InitStruct, uint32_t baudrate);

uint32_t UARTReceive(LPC_UART_TypeDef *UARTPort, uint8_t *rxbuf, uint8_t buflen);
uint32_t UARTSend(LPC_UART_TypeDef *UARTPort, uint8_t *txbuf, uint8_t buflen);
uint32_t UARTSend_AC(LPC_UART_TypeDef *UARTx, uint8_t *txbuf, uint8_t buflen);
uint32_t UARTSendString(LPC_UART_TypeDef *UARTPort, char *string);
void print_menu(LPC_UART_TypeDef *UARTx);

int User_sendchar (int ch);
uint8_t UARTNumExtract(LPC_UART_TypeDef *UARTx);


#endif /* __USER_UART_H */

/* --------------------------------- End Of File ------------------------------ */
/************************** PRIVATE FUNCTIONS *************************/
