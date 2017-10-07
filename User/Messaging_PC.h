#include "stdint.h"

#define TX_BUFFER_SIZE 					250
#define RX_BUFFER_SIZE 					250

#define msgRecieve_Sync1_PC 				0xAC
#define msgRecieve_Sync2_PC 				0xBD

#define msgTrans_Sync1						0xAC
#define msgTrans_Sync2						0xBD
#define msgTrans_RF_ID						0xAD
#define msgTrans_PC_ID						0xFE
#define msgTrans_PC_Rot_ID				0xA0


#define msgTrans_SetHigh_SUM_TX_ID			0xA6
#define msgTrans_SetLow_SUM_TX_ID				0xA7
#define msgTrans_SetHigh_Delta_TX_ID		0xA8
#define msgTrans_SetLow_Delta_TX_ID			0xA9

#define Message_ID_PIDGain						0x13
#define Message_ID_SetPoint_Pos				0x15
#define Message_ID_SetPoint_Spe				0x17
#define Message_ID_SinusGenStart			0x19

#define Message_ID_SendData						0x90
#define Message_ID_Stop								0x11

#define TRANS_HEADER_NO_PC						6

void parse_Message_PC(uint8_t* msg, uint8_t Message_Length, uint8_t Message_ID);
extern void parseByte_PC(unsigned char Data);
extern char tx_buffer_PC[TX_BUFFER_SIZE];
extern unsigned char tx_wr_index_PC,tx_rd_index_PC,tx_counter_PC;

extern uint32_t Message_Index_PC;
extern char Message_ID_PC;
extern char Message_Length_PC;
extern unsigned char Transmit_Check_Sum_PC;
uint8_t* Make_Trans_Msg_PC2Rot(uint8_t _id, uint8_t* _msg, uint8_t _lenght); 

typedef enum
{
	msgUNINIT_PC,
	msgGOT_SYNC1_PC,
	msgGOT_SYNC2_PC,
	msgGOT_ID_PC,
	msgGOT_LEN_PC,
	msgGOT_PAYLOAD_PC,
} Message_Status_t_PC;

typedef union 
{
	char			MessageChar[RX_BUFFER_SIZE];
	int16_t				MessageInt[RX_BUFFER_SIZE/2];
	float			MessageFloat[RX_BUFFER_SIZE/4];
	int				MessageInt32[RX_BUFFER_SIZE/4];
} Message_t_PC;

extern Message_Status_t_PC  Message_Status_PC;
extern Message_t_PC	Message_PC;
