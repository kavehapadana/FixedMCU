#include "AC_MotorDriverMsg.h"
//#include "my_serial.h"
#define SIZE_CRC	6

uint8_t	Send_MSG[AC_MSG_LENGTH];
uint8_t output_crc[4];
uint8_t input_crc[SIZE_CRC];

uint8_t* AC_Stop_msg(void)
{
	return	make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_MOVING_ADD_HIGH,MSG_MOVING_ADD_LOW,
								MSG_MOVING_STOP_HIGH,MSG_MOVING_STOP_LOW);
}

uint8_t* AC_Forward_msg(void)
{
		return	make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_MOVING_ADD_HIGH,MSG_MOVING_ADD_LOW,
								MSG_MOVING_FORWARD_HIGH,MSG_MOVING_FORWARD_LOW);
}
uint8_t* AC_Reverse_msg(void)
{
		return	make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_MOVING_ADD_HIGH,MSG_MOVING_ADD_LOW,
								MSG_MOVING_REVERSE_HIGH,MSG_MOVING_REVERSE_LOW);
}

uint8_t* AC_Freq_msg(uint16_t	_freq)
{
	if(_freq < 500)
		_freq = 500;
	if(_freq > 5000)
		_freq = 5000;
		
	return	make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_SETTING_ADD_FREQ_HIGH,MSG_SETTING_ADD_FREQ_LOW,
								(_freq >> 8),(_freq & 0xFF));
}
uint8_t* AC_Accel_msg(uint16_t	_accel)
{
	return	make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_SETTING_ADD_ACC_HIGH,MSG_SETTING_ADD_ACC_LOW,
								(_accel >> 8),(_accel & 0xFF));
}
uint8_t* AC_Decel_msg(uint16_t	_decel)
{
		return make_AC_Msg(MSG_SYNC1,MSG_SYNC2,
								MSG_SETTING_ADD_DEC_HIGH,MSG_SETTING_ADD_DEC_LOW,
								(_decel >> 8),(_decel & 0xFF));
}

uint8_t*  make_AC_Msg(uint8_t	sync1, uint8_t	sync2,
										uint8_t	add_high, uint8_t	add_low,
										uint8_t	val_high,uint8_t	val_low)
{
	Send_MSG[0] = input_crc[0] = sync1;
	Send_MSG[1] = input_crc[1] = sync2;
	Send_MSG[2] = input_crc[2] = add_high;
	Send_MSG[3] = input_crc[3] = add_low;
	Send_MSG[4] = input_crc[4] = val_high;
	Send_MSG[5] = input_crc[5] = val_low;
	Calculate_CRC(input_crc,SIZE_CRC);
	Send_MSG[6] = output_crc[0];
	Send_MSG[7] = output_crc[1];
	Send_MSG[8] = output_crc[2];
	Send_MSG[9] = output_crc[3];
        return Send_MSG;
	// AC_Driver_Transmit(Send_MSG, 10);
}


void Calculate_CRC(uint8_t* inputs, int size)
{
		uint32_t	crc = 0xFFFF;
		int pos = 0,i = 8;
		for(pos = 0;pos < size;pos++)
		{
			crc ^= (uint32_t) inputs[pos];
			for(i = 8;i != 0; i--)
			{
				if((crc & 0x0001) != 0)
				{
					crc >>= 1;
					crc ^= 0xA001;
				}
				else
				{crc >>= 1;}
			}
		}
		output_crc[0] = crc & 0xFF;
		output_crc[1] = (crc >> 8) & 0xFF;
		output_crc[2] = (crc >> 16) & 0xFF;
		output_crc[3] = (crc >> 24) & 0xFF;			
}
