#include "stdint.h"

#define TX_BUFFER_SIZE 					250
#define RX_BUFFER_SIZE 					250

#define msgRecieve_Sync1 				0x7B
#define msgRecieve_Sync2 				0xEA

#define msgTrans_Sync1					0xAC
#define msgTrans_Sync2					0xBD
#define msgTrans_RF_ID					0xAD
#define msgTrans_PC_ID					0xFE

#define Message_ID_PIDGain					0x13
#define Message_ID_SetPoint_Pos			0x15
#define Message_ID_SetPoint_Spe			0x17
#define Message_ID_SinusGenStart		0x19

#define Message_ID_SendData				0x90
#define Message_ID_Stop						0x11
#define msgAliveConnection_ID			0xFB
#define TRANS_HEADER_NO_RF				6

void parse_Message(uint8_t* msg, uint8_t Message_Length, uint8_t Message_ID);
void parseByte(unsigned char Data);
uint8_t* Make_Trans_Msg(uint8_t _id, uint8_t* _msg, uint8_t _lenght); 
extern char tx_buffer[TX_BUFFER_SIZE];
extern unsigned char tx_wr_index,tx_rd_index,tx_counter;

extern char Receive_Check_Sum;
extern uint32_t Message_Index;
//extern char Message_ID;
//extern char Message_Length;
extern unsigned char Transmit_Check_Sum;
extern uint8_t flageMCU_RotaryAlive;
extern uint8_t CntMCU_RotaryAlive;
typedef enum
{
	msgUNINIT,
	msgGOT_SYNC1,
	msgGOT_SYNC2,
	msgGOT_ID,
	msgGOT_LEN,
	msgGOT_PAYLOAD,
} Message_Status_t;

typedef enum
{
	AC_STOP = 0x50,
	AC_FORWARD,
	AC_REVERSE,
	AC_EGC_STOP,
	AC_PARK,
	AC_FREQ,
	AC_ACC,
	AC_DEC
} AC_ID_t;
typedef union 
{
	uint8_t			MessageChar[RX_BUFFER_SIZE];
	int16_t				MessageInt[RX_BUFFER_SIZE/2];
	float			MessageFloat[RX_BUFFER_SIZE/4];
	int				MessageInt32[RX_BUFFER_SIZE/4];
} Message_t;

extern Message_Status_t  Message_Status;
extern Message_t	Message;
extern AC_ID_t AC_ID;
