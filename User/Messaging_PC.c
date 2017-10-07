#include "Messaging_PC.h"
#include "main.h"
uint32_t Message_Index_PC;	
char Message_ID_PC;
char Message_Length_PC;

Message_Status_t_PC 	Message_Status_PC;
Message_t_PC			Message_PC;
uint8_t _msg_PC[RX_BUFFER_SIZE];
uint8_t _msg_trans_PC[RX_BUFFER_SIZE];
#define CRC16 0x8005

uint16_t gen_crc16_PC2Rot(const uint8_t *data, uint16_t size);
union Data_16_8
{
  uint16_t u16;
  uint8_t u8[2];
}u8_16_PC;
char cntCRC = 0;
	void parseByte_PC(unsigned char Data)
{

		switch(Message_Status_PC)
	{
		case msgUNINIT_PC:
		if (Data == msgRecieve_Sync1_PC)
		{
			Message_Status_PC++;
		}
		break;
		
		case msgGOT_SYNC1_PC:
		if (Data != msgRecieve_Sync2_PC)
		{
			goto restart;
		}
		Message_Index_PC = 0;
		Message_Status_PC++;
		break;		
		case msgGOT_SYNC2_PC:
		Message_ID_PC = Data;
		Message_Status_PC++;
		break;
		
		case msgGOT_ID_PC:
		Message_Length_PC = Data;
		Message_Status_PC++;
		break;
		
		case msgGOT_LEN_PC:
		Message_PC.MessageChar[Message_Index_PC] = Data;
		Message_Index_PC++;
		if (Message_Index_PC >= Message_Length_PC) {
			Message_Status_PC++;
			cntCRC = 0;
		}
		break;
		
		case msgGOT_PAYLOAD_PC:
			switch(cntCRC)
			{
				case 0:
					u8_16_PC.u8[0] = Data;
					break;
				case 1:
					u8_16_PC.u8[1] = Data;
					break;
			}
			cntCRC++;
			if(cntCRC == 2)
			{
				cntCRC = 0;
				memcpy(_msg_PC,Message_PC.MessageChar,(unsigned int)RX_BUFFER_SIZE);
				if(u8_16_PC.u16 != gen_crc16_PC2Rot(_msg_PC,Message_Length_PC))
					goto error;
				parse_Message_PC(_msg_PC, Message_Length_PC, Message_ID_PC);
			}
			else
				break;
		goto restart; 
	}
	return;	
	error:
	{
		//GPIO_ToggleBits(GPIOC, GPIO_Pin_5);
	}
	restart: Message_Status_PC = msgUNINIT_PC;
	return;
}

uint8_t* Make_Trans_Msg_PC2Rot(uint8_t _id, uint8_t* msg_, uint8_t _lenght)
{
	uint8_t chksum = 0,cnt; 
	_msg_trans_PC[0] = msgTrans_Sync1;
	_msg_trans_PC[1] = msgTrans_Sync2;

	_msg_trans_PC[2] = _id;
	chksum += _id;
	_msg_trans_PC[3] = _lenght;
	chksum += _lenght;
	
	for(cnt = 0;cnt < _lenght;cnt++)
	{
		_msg_trans_PC[cnt+4] = msg_[cnt];
		chksum += msg_[cnt];
	}
	u8_16_PC.u16 = gen_crc16_PC2Rot(msg_,_lenght);
	_msg_trans_PC[_lenght + 4] = u8_16_PC.u8[0];
	_msg_trans_PC[_lenght + 5] = u8_16_PC.u8[1];
	
	return _msg_trans_PC;
}

uint16_t gen_crc16_PC2Rot(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;
    int i;
    uint16_t crc = 0;
    int j = 0x0001;

    /* Sanity check: */
    if(data == (void *)0)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }


    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    i = 0x8000;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}
