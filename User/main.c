/* Includes ------------------------------------------------------------------- */
#define  GLOBAL_VAR
#define  GLOBAL_PINS
#define  IMPORTANT_VAR
#define  Functions
#define	 USRATS_DEFINES
#define  VARIABLES
#define	 MY_TEST
#include "lpc17xx_libcfg.h"
#include "lpc17xx_timer.h"
#include "User_Uart.h"
#include "lpc17xx_pinsel.h"
#include "User_GPIO.h"
#include "MASK.h"
#include "Messaging.h"
#include "Messaging_PC.h"
#include "AC_MotorDriverMsg.h"
#include <stdio.h>
#include "lpc17xx_exti.h"


//timer initialization
TIM_TIMERCFG_Type TIM_ConfigStruct;
TIM_MATCHCFG_Type TIM_MatchConfigStruct ;
uint8_t volatile timer0_flag = FALSE, timer1_flag = FALSE;
FunctionalState LEDStatus = ENABLE;

/************************** PRIVATE FUNCTION *************************/

#ifdef GLOBAL_VAR
#ifdef GLOBAL_PINS

//#define PC_INT_PRIORITY       10
//#define RF_INT_PRIORITY       26
//#define AC_MOTOR_INT_PRIORITY 14
//#define PROXY_INT_PRIORITY    15
//#define TIMER0_INT_PRIORITY   29
//#define TIMER1_INT_PRIORITY   9

#define PC_INT_PRIORITY       15
#define RF_INT_PRIORITY       16
#define AC_MOTOR_INT_PRIORITY 17
#define PROXY_INT_PRIORITY    11
#define TIMER0_INT_PRIORITY   10
#define TIMER1_INT_PRIORITY   20

// Declare Push Button Pins
pin Button_1 = {0,7,0};
pin Button_2 = {0,6,0};
pin Button_3 = {0,5,0};
pin Button_4 = {0,4,0};

// Declare LED Pins
pin LED_1 = {1,0,1};
pin LED_2 = {1,1,1};
pin LED_3_rcvRF = {1,4,1};
pin LED_4_trs2PC = {1,8,1};
pin LED_5 = {1,9,1};
pin LED_6 = {1,10,1};
pin LED_7 = {1,14,1};
pin LED_8 = {1,15,1};

// Declare Encoder Pins
pin encoderData = {0,8,0};
pin encoderClock = {0,9,1};

// Declare External INT for Proxy Sensor
pin proxyPin = {0,25,0};

uint8_t proxyStateFlag;

uint8_t motorStartFlag;
uint8_t motorStateFlag;

#endif
#ifdef IMPORTANT_VAR
#define TRANS_MSG_RF_SIZE			100
	
uint8_t aliveConnectionFlg = 0;
#endif
#ifdef	VARIABLES
typedef enum
{
	e_Rot_COM_RF = 0,
	e_Rot_COM_SUM,
	e_Rot_COM_Delta,
	e_Not1,
	e_Fix_COM_RF,
	e_Fix_COM_PC,
	e_Fix_COM_AC_M,	
	e_Not2,
} Error_Status_t;	
	Error_Status_t er_status;	
	
int Cnt_timer0 = 0,Cnt_timer1 = 0;
union Data_16_8
{
  uint16_t u16;
  uint8_t u8[2];
}u8to16;

int Cnt = 0; char ReadDataFlg = 1;
uint16_t Degree = 0, Rev = 0,RefDegree,RefRev;
unsigned long Encoder_Data = 0;
char temp[5];
int tempCnt = 0;
#endif

#endif
#ifdef Functions
	void external_Interrupt_Fnc(uint8_t _mask);
	void TIMER0_IRQHandler(void);
	void TIMER1_IRQHandler(void);
	void Init_Timer0(void);
	void Init_Timer1(void);
	void IO_Initialling(void);
	void Encoder_Initialling(void);
	void Trans_2PC_Message(void);
	void Serials_Init(void);
/* Interrupt service routine */
	void TIMER0_IRQHandler(void);
#endif
#ifdef  USRATS_DEFINES
#define UART_RF            		  			UART2
#define UART_LPC_RF             			LPC_UART2
#define UART_RF_BAUDRATE   						19200
#define	UART_RF_IRQHandler	 					UART2_IRQHandler

#define UART_PC            						UART1
#define UART_LPC_PC         					LPC_UART1
#define UART_PC_BAUDRATE   						38400
#define	UART_PC_IRQHandler						UART1_IRQHandler

#define UART_AC_Motor            			UART0
#define UART_LPC_AC_Motor         		LPC_UART0
#define UART_AC_Motor_BAUDRATE   			9600
#define	UART_AC_Motor_IRQHandler			UART0_IRQHandler

#define UART_Not_Assigned            	UART3
#define UART_LPC_Not_Assigned         LPC_UART3
#define UART_Not_Assigned_BAUDRATE   	19200
#define	UART_Not_Assigned_IRQHandler	UART3_IRQHandler

#endif
uint8_t nn[2] = {0,0};
uint8_t mn[8] = {0x23};
uint32_t PC_Sent_Interval_cnt;

uint8_t myData[100];
int maxBuff = 300;
int rcvLen = 0;
int cntBuffLen = 0;

int main(void)
{
  IO_Initialling();  
  Encoder_Initialling();  
  Serials_Init();
  Init_Timer0();
  Init_Timer1();
  
  GPIOINT_Init(&proxyPin, 3, PROXY_INT_PRIORITY, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE);
  //parse_Message_PC(nn,2,AC_STOP);
  //parse_Message_PC(nn,2,AC_FORWARD);
  
  proxyStateFlag = 0; 
  motorStateFlag = 0;
  motorStartFlag = 0;
   
  PC_Sent_Interval_cnt = 0;
 
  while(1)
  {
		rcvLen = UARTReceive(UART_RF,myData,maxBuff);
		cntBuffLen = 0;
		pinOn(&LED_3_rcvRF);
		while(cntBuffLen < rcvLen)
		{
			pinOff(&LED_3_rcvRF);
			parseByte(myData[cntBuffLen]);
			cntBuffLen++;
		}
		
		rcvLen = UARTReceive(UART_PC,myData,maxBuff);
		cntBuffLen = 0;
		while(cntBuffLen < rcvLen)
		{
			parseByte_PC(myData[cntBuffLen]);
			cntBuffLen++;
		}	
		
		if(pinStatus(&Button_4) && motorStateFlag)
		{
			if(motorStartFlag == 1)
			{
				UARTSend(UART_AC_Motor,AC_Forward_msg(),AC_MSG_LENGTH);
			}
			else
			{
				UARTSend(UART_AC_Motor,AC_Stop_msg(),AC_MSG_LENGTH);
			}
			motorStateFlag = 0;
		}
		
		if((!pinStatus(&Button_4)) && (!motorStateFlag))
		{
			motorStateFlag = 1;
			motorStartFlag^= 1;
//			if(motorStartFlag == 1)
//			{
//				pinOff(&LED_4_trs2PC);
//			}
//			else
//			{
//				pinOn(&LED_4_trs2PC);
//			}
		}		
  }
}

void Serials_Init(void)
{
  Com_Init(UART_RF,ENABLE,DISABLE,RF_INT_PRIORITY,UART_RF_BAUDRATE,1);
  Com_Init(UART_PC,ENABLE,DISABLE,PC_INT_PRIORITY,UART_PC_BAUDRATE,1);
  Com_Init(UART_AC_Motor,ENABLE,DISABLE,AC_MOTOR_INT_PRIORITY,UART_AC_Motor_BAUDRATE,1); 	
}

AC_ID_t AC_ID;
uint8_t msgv[2] = {2,3};
uint16_t	u16_temp;

int oneSendInTwo = 0;
void parse_Message(uint8_t* msg, uint8_t Message_Length, uint8_t Message_ID)
{
	if(Message_ID == msgTrans_RF_ID)		
	{
			NVIC_EnableIRQ(TIMER0_IRQn);
			while(ReadDataFlg == 1);
			NVIC_DisableIRQ(TIMER0_IRQn);
			Degree = (Encoder_Data & 0x0FFF);
			Rev = (Encoder_Data >> 12);
			Encoder_Data = 0;
			ReadDataFlg = 1;
			
			msg[Message_Length]     = (Degree & 0xFF);
			msg[Message_Length + 1] = (Degree >> 8);
			msg[Message_Length + 2] = (Rev & 0xFF);
			msg[Message_Length + 3] = (Rev >> 8);	
			msg[Message_Length + 4] = (RefDegree & 0xFF);
			msg[Message_Length + 5] = (RefDegree >> 8);
			msg[Message_Length + 6] = (RefRev & 0xFF);
			msg[Message_Length + 7] = (RefRev >> 8);
			//if(!((oneSendInTwo++)%300))	
			UARTSend(UART_PC, Make_Trans_Msg(msgTrans_PC_ID,msg,Message_Length + 8) , (Message_Length + 8 + TRANS_HEADER_NO_RF));
			pinOff(&LED_4_trs2PC);
	}
}

void parse_Message_PC(uint8_t* msg, uint8_t Message_Length, uint8_t Message_ID)
{
	if(Message_ID == msgTrans_PC_Rot_ID)		
	{
			UARTSend(UART_RF, Make_Trans_Msg_PC2Rot(msgTrans_PC_Rot_ID,msg,Message_Length) , (TRANS_HEADER_NO_PC + Message_Length));
	}
	if(Message_ID == msgAliveConnection_ID)
	{
		aliveConnectionFlg = 1;
	}
	switch(Message_ID)
	{
		case AC_STOP:
			UARTSend(UART_AC_Motor,AC_Stop_msg(),AC_MSG_LENGTH);
			//Send_uint_AC(AC_Stop_msg(),AC_MSG_LENGTH);	
			break;
		
		case AC_FORWARD:
			UARTSend(UART_AC_Motor,AC_Forward_msg(),AC_MSG_LENGTH);			
			//Send_uint_AC(AC_Forward_msg(),AC_MSG_LENGTH);
			break;
		
		case AC_REVERSE:
			UARTSend(UART_AC_Motor, AC_Reverse_msg(),AC_MSG_LENGTH);		
			//Send_uint_AC(AC_Reverse_msg(),AC_MSG_LENGTH);	
			break;
		
		case AC_EGC_STOP:
			UARTSend(UART_AC_Motor, msgv,2);		
			//Send_uint_AC(msgv,2);
			break;
		
		case AC_PARK:			
			break;
		
		case AC_FREQ:
			u16_temp = msg[0] + msg[1] * 256; 			
			UARTSend(UART_AC_Motor,AC_Freq_msg(u16_temp),AC_MSG_LENGTH);			
			//Send_uint_AC(AC_Freq_msg(u16_temp),AC_MSG_LENGTH);
		break;
		
		case AC_ACC:
			u16_temp = msg[0] + msg[1] * 256;
			UARTSend(UART_AC_Motor,AC_Accel_msg(u16_temp) ,AC_MSG_LENGTH);	
		//Send_uint_AC(AC_Accel_msg(u16_temp),AC_MSG_LENGTH);	
			break;
		
		case AC_DEC:
			u16_temp = msg[0] + msg[1] * 256;
			UARTSend(UART_AC_Motor,AC_Decel_msg(u16_temp) ,AC_MSG_LENGTH);							
			//Send_uint_AC(AC_Decel_msg(u16_temp),AC_MSG_LENGTH);
			break;
	}
}

void TIMER0_IRQHandler(void)
{	 
	if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)== SET)
	{
		TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
		Cnt_timer0++;		
		if(!(Cnt_timer0%200))
			pinToggle(&LED_8);

	
		if(ReadDataFlg)
		{
			if(Cnt%2)
			{
				pinOn(&encoderClock);
			}
			else
			{
				pinOff(&encoderClock);
				if(Cnt!=0)
				{  
					Encoder_Data <<= 1;
					Encoder_Data |= pinStatus(&encoderData);
				}
			}
			Cnt++;
			if(Cnt > 57)
			{
				Cnt = 0;
				ReadDataFlg = 0;
			}
		}
	}	
}

int cntTimer_1 = 0;
uint32_t CntTimer1_LED = 0;

char flageLED =0;
uint8_t kkTest[1] = {0x44};
void TIMER1_IRQHandler(void)
{	 
	if (TIM_GetIntStatus(LPC_TIM1, TIM_MR0_INT)== SET)
	{
		TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
		
		if(!flageMCU_RotaryAlive)
		{
			if(++CntMCU_RotaryAlive == 7)
			{
				CntMCU_RotaryAlive = 0;
				mn[1] = 0xF8;
				parse_Message(mn,9,0xAD);
			}
		}
		CntTimer1_LED++;
		if(!(CntTimer1_LED % 80))
		{
			//CntTimer1_LED = 0;
			flageMCU_RotaryAlive = 0;
			pinToggle(&LED_2);
			pinOn(&LED_4_trs2PC);
			//UARTSend(UART_RF,kkTest,1);
			CntMCU_RotaryAlive = 0;
			//parse_Message(mn,1);
			//UARTSend(UART_RF, mn , 1);
		}
		if(cntTimer_1++ > 10000)
		{
			cntTimer_1 = 0;
			if(!aliveConnectionFlg)
			{
				parse_Message_PC(nn,2,AC_STOP);
			}			
			aliveConnectionFlg = 0;
		}
		if(ReadDataFlg)
		{
//				ReadDataFlg = 0;	
		}
		else
		{	
//			ReadDataFlg = 1;
//			//parse_Message(mn,1);
//			Degree = (Encoder_Data & 0x0FFF);
//			Rev = (Encoder_Data >> 12);
//			Encoder_Data = 0;
//			pinToggle(&LED_7);
		}
	}
}

void Init_Timer0(void)
{
	// Initialize 4 timers, prescale count time of 100uS
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
		// Configure 4 match channels
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value
	TIM_MatchConfigStruct.MatchValue   = 4 - 1;
	// Set configuration for Tim_MatchConfig
	TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);
		/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER0_IRQn, TIMER0_INT_PRIORITY);
	/* Enable interrupt for timer 0 */
//	NVIC_EnableIRQ(TIMER0_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM0,ENABLE);
}

void Init_Timer1(void)
{
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL; // In this state the timer is 1 MHz
	TIM_ConfigStruct.PrescaleValue	= 250; // 1 MHz / 250 = 4 KHz

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE,&TIM_ConfigStruct);
		// Configure 4 match channels
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if MR0 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Stop on MR0 if MR0 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//Toggle MR0 pin if MR0 matches it
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value
	TIM_MatchConfigStruct.MatchValue   = 50 - 1; // x - 1 : x is matchvalue // 4 KHz / 5 = 800 Hz
	// Set configuration for Tim_MatchConfig
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);
		/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(TIMER1_IRQn, TIMER1_INT_PRIORITY);
	/* Enable interrupt for timer 0 */
	NVIC_EnableIRQ(TIMER1_IRQn);
	// To start timer 0
	TIM_Cmd(LPC_TIM1,ENABLE);
}

void IO_Initialling(void)
{
	
  pinConfig(&LED_1);
  pinConfig(&LED_2);
  pinConfig(&LED_3_rcvRF);
  pinConfig(&LED_4_trs2PC);
  pinConfig(&LED_5);
  pinConfig(&LED_6);
  pinConfig(&LED_7);
  pinConfig(&LED_8);
  
  pinConfig(&Button_1);
  pinConfig(&Button_2);
  pinConfig(&Button_3);
  pinConfig(&Button_4);
  
  pinOn(&LED_1);
  pinOn(&LED_2);
//  pinOn(&LED_3_rcvRF);
  pinOn(&LED_4_trs2PC);
  pinOn(&LED_5);
  pinOn(&LED_6);
  pinOn(&LED_7);
  pinOn(&LED_8);
//  
//  pinOff(&LED_1);
//  pinOff(&LED_2);
//  pinOff(&LED_3_rcvRF);
//  pinOff(&LED_4_trs2PC);
//  pinOff(&LED_5);
//  pinOff(&LED_6);
//  pinOff(&LED_7);
//  pinOff(&LED_8);
}

void Encoder_Initialling(void)
{
  pinConfig(&encoderData);
  pinConfig(&encoderClock);
  pinOn(&encoderClock);
  pinOff(&encoderData);
}



/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/

uint8_t dataBuf[200];
int ind = 0;
void UART_RF_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId((LPC_UART_TypeDef *)UART_LPC_RF);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus((LPC_UART_TypeDef *)UART_LPC_RF);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}

	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
	if ((((LPC_UART_TypeDef *)UART_RF)->LSR & UART_LSR_RDR)) {
				
				//uint8_t data = 	UART_ReceiveByte(UART_RF);
				//parseByte(data);
				UART_IntReceive((LPC_UART_TypeDef *)UART_RF);
		
			}
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit((LPC_UART_TypeDef *)UART_LPC_RF);
	}
}

void UART_PC_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId((LPC_UART_TypeDef *)UART_LPC_PC);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus((LPC_UART_TypeDef *)UART_LPC_PC);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}
	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			if ((((LPC_UART_TypeDef *)UART_PC)->LSR & UART_LSR_RDR)) {
						//parseByte_PC(UART_ReceiveByte(UART_PC));
				UART_IntReceive((LPC_UART_TypeDef *)UART_PC);
			}	
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit((LPC_UART_TypeDef *)UART_LPC_PC);
	}
}
void UART_AC_Motor_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId((LPC_UART_TypeDef *)UART_LPC_AC_Motor);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus((LPC_UART_TypeDef *)UART_LPC_AC_Motor);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}
	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			if ((((LPC_UART_TypeDef *)UART_AC_Motor)->LSR & UART_LSR_RDR)) {
					//	Sensors_UARTs_Data[1] = UART_ReceiveByte(UART_AC_Motor);
			}	
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit((LPC_UART_TypeDef *)UART_LPC_AC_Motor);
	}
}

void UART_Not_Assigned_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1;

	/* Determine the interrupt source */
	intsrc = UART_GetIntId(UART_LPC_Not_Assigned);
	tmp = intsrc & UART_IIR_INTID_MASK;

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS){
		// Check line status
		tmp1 = UART_GetLineStatus(UART_LPC_Not_Assigned);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
				| UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1) {
				UART_IntErr(tmp1);
		}
	}
	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
			if ((UART_Not_Assigned->LSR & UART_LSR_RDR)) {
					//	Sensors_UARTs_Data[2] = UART_ReceiveByte(UART_Not_Assigned);
			}	
	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE){
			UART_IntTransmit((LPC_UART_TypeDef *)UART_Not_Assigned);
	}
}


void EINT3_IRQHandler(void)
{
  if (GPIO_IntCheck(&proxyPin, EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE))
  {
		RefRev = Rev;
		RefDegree = Degree;		
  }
}
