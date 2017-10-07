
#include "User_Uart.h"
#include "debug_frmwrk.h"
#include "Messaging.h"
#include "Messaging_PC.h"
#include <stdio.h>


UART_RING_BUFFER_T rb[4];
__IO FlagStatus TxIntStat[4];

unsigned long systickcnt = 0;
extern volatile unsigned long SysTickCnt;

void UART_IntReceive(LPC_UART_TypeDef *UARTx)
{
	uint8_t tmpc;
	uint32_t rLen;
  uint8_t portNumber = UARTNumExtract(UARTx);

	while(1){
		// Call UART read function in UART driver
		rLen = UART_Receive(UARTx, &tmpc, 1, NONE_BLOCKING);
		
		// If data received
		if (rLen){
			/* Check if buffer is more space
			 * If no more space, remaining character will be trimmed out
			 */
			if ((!__BUF_IS_FULL(rb[portNumber].rx_head,rb[portNumber].rx_tail))||(rb[portNumber].RxBufferRingEn)){
				rb[portNumber].rx[rb[portNumber].rx_head] = tmpc;
				__BUF_INCR(rb[portNumber].rx_head);
			}
		}
		// no more data
		else {
			break;
		}
	}
}

/********************************************************************//**
 * @brief 	UART transmit function (ring buffer used)
 * @param[in]	UARTx	Selected UART peripheral used to Transmit data,
 * @return 	None
 *********************************************************************/
void UART_IntTransmit(LPC_UART_TypeDef *UARTx)
{
  uint8_t portNumber = UARTNumExtract(UARTx);
  // Disable THRE interrupt    
	UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);
    

	/* Wait for FIFO buffer empty, transfer UART_TX_FIFO_SIZE bytes
	 * of data or break whenever ring buffers are empty */
	/* Wait until THR empty */
    while (UART_CheckBusy(UARTx) == SET);

	while (!__BUF_IS_EMPTY(rb[portNumber].tx_head,rb[portNumber].tx_tail))
    {
        /* Move a piece of data into the transmit FIFO */
    	if (UART_Send(UARTx, (uint8_t *)&rb[portNumber].tx[rb[portNumber].tx_tail], 1, NONE_BLOCKING)){
        /* Update transmit ring FIFO tail pointer */
        __BUF_INCR(rb[portNumber].tx_tail);
    	} else {
    		break;
    	}
    }

    /* If there is no more data to send, disable the transmit
       interrupt - else enable it or keep it enabled */
	if(__BUF_IS_EMPTY(rb[portNumber].tx_head, rb[portNumber].tx_tail)) 
		{
    	UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);
    	// Reset Tx Interrupt state
    	TxIntStat[portNumber] = RESET;
    }
    else
		{
      // Set Tx Interrupt state
			TxIntStat[portNumber] = SET;
    	UART_IntConfig(UARTx, UART_INTCFG_THRE, ENABLE);
    }
}

/********************************************************************//**
 * @brief 	UART transmit function (ring buffer used)
 * @param[in]	UARTx	Selected UART peripheral used to Transmit data,
 * @return 	None
 *********************************************************************/
void UART_IntErr(uint8_t bLSErrType)
{
	//while (1){}
}


/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************//**
 * @brief	UART Initialization Function Version_1
 * @param[in]	UARTx         Selected UART peripheral used to Initial,
 * @param[in]	transmitEn    Tranmition Enable,
 * @param[in]	Priority      UART Interrupt Priority,
 * @param[in]	baudrate      UART baudrate,
 * @return 	None
 **********************************************************************/
void Com_Init(LPC_UART_TypeDef *UARTx,FunctionalState transmitEn, uint8_t interruptEn, uint32_t Priority, uint32_t baudrate, uint8_t RxBufferRingEn)
{

  // UART Configuration structure variable
  UART_CFG_Type UARTConfigStruct;
  // UART FIFO configuration Struct variable
  UART_FIFO_CFG_Type UARTFIFOConfigStruct;
  // Pin configuration
  PINSEL_CFG_Type PinCfg;
  
  uint8_t portNumber = UARTNumExtract(UARTx);


  /* Initialize UART Configuration parameter structure:
   * Baudrate = baudrate bps
   * 8 data bit
   * 1 Stop bit
   * None parity
   */
  
  UART_ConfigStruct(&UARTConfigStruct,baudrate);
  
  /* Initialize FIFOConfigStruct to default state:
   * 				- FIFO_DMAMode = DISABLE
   * 				- FIFO_Level = UART_FIFO_TRGLEV0
   * 				- FIFO_ResetRxBuf = ENABLE
   * 				- FIFO_ResetTxBuf = ENABLE
   * 				- FIFO_State = ENABLE
   */
  UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

  
  /*
     * Initialize UART0 pin connect
     */
  if ((LPC_UART_TypeDef *) UARTx == LPC_UART0)
  {
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 2;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 3;
    PINSEL_ConfigPin(&PinCfg);
  }
  
  /*
     * Initialize UART1 pin connect
     */
  if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1)
  {
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 16;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 15;
    PINSEL_ConfigPin(&PinCfg);
  }

    /*
     * Initialize UART2 pin connect
     */
  if ((LPC_UART_TypeDef *) UARTx == LPC_UART2)
  {  
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 10;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 11;
    PINSEL_ConfigPin(&PinCfg);
  }

    /*
     * Initialize UART3 pin connect
     */
  if ((LPC_UART_TypeDef *) UARTx == LPC_UART3)
  {
    PinCfg.Funcnum = 2;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
  }

    // Initialize UART0 peripheral with given to corresponding parameter
    UART_Init((LPC_UART_TypeDef *)UARTx, &UARTConfigStruct);
    // Initialize FIFO for UART0 peripheral
    UART_FIFOConfig((LPC_UART_TypeDef *)UARTx, &UARTFIFOConfigStruct);
    // Enable UART Transmit
    UART_TxCmd((LPC_UART_TypeDef *)UARTx, transmitEn);

    /* Enable UART Rx interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)UARTx, UART_INTCFG_RBR, ENABLE);
    /* Enable UART line status interrupt */
    UART_IntConfig((LPC_UART_TypeDef *)UARTx, UART_INTCFG_RLS, ENABLE);
    /*
     * Do not enable transmit interrupt here, since it is handled by
     * UART_Send() function, just to reset Tx Interrupt state for the
     * first time
     */
    TxIntStat[portNumber] = RESET;
  
  // Reset ring buf head and tail idx
  rb[portNumber].RxBufferRingEn = RxBufferRingEn;
  __BUF_RESET(rb[portNumber].rx_head);
  __BUF_RESET(rb[portNumber].rx_tail);
  __BUF_RESET(rb[portNumber].tx_head);
  __BUF_RESET(rb[portNumber].tx_tail);

//  // Reset ring buf head and tail idx
//  rb_AC[portNumber].RxBufferRingEn = RxBufferRingEn;
//  __BUF_RESET(rb_AC[portNumber].rx_head);
//  __BUF_RESET(rb_AC[portNumber].rx_tail);
//  __BUF_RESET(rb_AC[portNumber].tx_head);
//  __BUF_RESET(rb_AC[portNumber].tx_tail);

  if ((LPC_UART_TypeDef *) UARTx == LPC_UART0)
  {
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(UART0_IRQn, Priority);
        /* Enable Interrupt for UART0 channel */
    NVIC_EnableIRQ(UART0_IRQn);
  }

  if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1)
  {
    NVIC_SetPriority(UART1_IRQn, Priority);
    NVIC_EnableIRQ(UART1_IRQn);
  }

  if ((LPC_UART_TypeDef *) UARTx == LPC_UART2)
  {
    NVIC_SetPriority(UART2_IRQn, Priority);
    NVIC_EnableIRQ(UART2_IRQn);
  }

  if ((LPC_UART_TypeDef *) UARTx == LPC_UART3)
  {
    NVIC_SetPriority(UART3_IRQn, Priority);
    NVIC_EnableIRQ(UART3_IRQn);
  }            
}


/*****************************************************************************//**
* @brief	Fills UART_InitStruct member;
* @param[in]	UART_InitStruct Pointer to a UART_CFG_Type structure
*                     which will be initialized,
* @return	None
*******************************************************************************/
void UART_ConfigStruct(UART_CFG_Type *UART_InitStruct, uint32_t baudrate)
{
  UART_InitStruct->Baud_rate = baudrate;
  UART_InitStruct->Databits = UART_DATABIT_8;
  UART_InitStruct->Parity = UART_PARITY_NONE;
  UART_InitStruct->Stopbits = UART_STOPBIT_1;
}



uint32_t UARTSend(LPC_UART_TypeDef *UARTx, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;
    uint8_t portNumber = UARTNumExtract(UARTx);
    
	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
    UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb[portNumber].tx_head, rb[portNumber].tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb[portNumber].tx[rb[portNumber].tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb[portNumber].tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat[portNumber] == RESET) {
		UART_IntTransmit(UARTx);
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		UART_IntConfig(UARTx, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}
uint32_t UARTSend_AC(LPC_UART_TypeDef *UARTx, uint8_t *txbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) txbuf;
    uint32_t bytes = 0;
    uint8_t portNumber = UARTNumExtract(UARTx);
    
	/* Temporarily lock out UART transmit interrupts during this
	   read so the UART transmit interrupt won't cause problems
	   with the index values */
    UART_IntConfig(UARTx, UART_INTCFG_THRE, DISABLE);

	/* Loop until transmit run buffer is full or until n_bytes
	   expires */
	while ((buflen > 0) && (!__BUF_IS_FULL(rb[portNumber].tx_head, rb[portNumber].tx_tail)))
	{
		/* Write data from buffer into ring buffer */
		rb[portNumber].tx[rb[portNumber].tx_head] = *data;
		data++;

		/* Increment head pointer */
		__BUF_INCR(rb[portNumber].tx_head);

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/*
	 * Check if current Tx interrupt enable is reset,
	 * that means the Tx interrupt must be re-enabled
	 * due to call UART_IntTransmit() function to trigger
	 * this interrupt type
	 */
	if (TxIntStat[portNumber] == RESET) {
		UART_IntTransmit(UARTx);
	}
	/*
	 * Otherwise, re-enables Tx Interrupt
	 */
	else {
		UART_IntConfig(UARTx, UART_INTCFG_THRE, ENABLE);
	}

    return bytes;
}


uint32_t UARTReceive(LPC_UART_TypeDef *UARTx, uint8_t *rxbuf, uint8_t buflen)
{
    uint8_t *data = (uint8_t *) rxbuf;
    uint32_t bytes = 0;
    uint8_t portNumber = UARTNumExtract(UARTx);

	/* Temporarily lock out UART receive interrupts during this
	   read so the UART receive interrupt won't cause problems
	   with the index values */
	UART_IntConfig(UARTx, UART_INTCFG_RBR, DISABLE);

	/* Loop until receive buffer ring is empty or
		until max_bytes expires */
	while ((buflen > 0) && (!(__BUF_IS_EMPTY(rb[portNumber].rx_head, rb[portNumber].rx_tail))))
	{
		/* Read data from ring buffer into user buffer */
		*data = rb[portNumber].rx[rb[portNumber].rx_tail];
		data++;

		/* Update tail pointer */
		__BUF_INCR(rb[portNumber].rx_tail); 

		/* Increment data count and decrement buffer size count */
		bytes++;
		buflen--;
	}

	/* Re-enable UART interrupts */
	UART_IntConfig(UARTx, UART_INTCFG_RBR, ENABLE);

    return bytes;
}


void Send_uint_AC(uint8_t* data,int len)
{
	int i = 0,j = 0;
	for(i = 0;i<len;i++)
	{
	 UART_Send((LPC_UART_TypeDef *)LPC_UART0, &data[i], 1, BLOCKING);
   for(j= 0; j<1000;j++)
		{}
	}
}



uint8_t UARTNumExtract(LPC_UART_TypeDef *UARTx)
{
  if ((LPC_UART_TypeDef *) UARTx == LPC_UART0)
  {
    return 0;
  }

  if ((LPC_UART1_TypeDef *) UARTx == LPC_UART1)
  {
    return 1;
  }

  if ((LPC_UART_TypeDef *) UARTx == LPC_UART2)
  {
    return 2;
  }

  if ((LPC_UART_TypeDef *) UARTx == LPC_UART3)
  {
    return 3;
  }
  return 0xFF;
}


