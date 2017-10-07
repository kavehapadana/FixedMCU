#include <stdint.h>

#define MSG_SYNC1		0x01
#define MSG_SYNC2   0x06

#define MSG_MOVING_ADD_HIGH			0x00
#define MSG_MOVING_ADD_LOW			0x05

#define MSG_MOVING_FORWARD_HIGH			0x00
#define MSG_MOVING_FORWARD_LOW			0x02

#define MSG_MOVING_STOP_HIGH		  	0x00
#define MSG_MOVING_STOP_LOW					0x01

#define MSG_MOVING_REVERSE_HIGH			0x00
#define MSG_MOVING_REVERSE_LOW			0x04

#define MSG_SETTING_ADD_FREQ_HIGH		0x80
#define MSG_SETTING_ADD_FREQ_LOW		0xFF

#define MSG_SETTING_ADD_ACC_HIGH		0x81
#define MSG_SETTING_ADD_ACC_LOW			0x00

#define MSG_SETTING_ADD_DEC_HIGH		0x81
#define MSG_SETTING_ADD_DEC_LOW			0x01

#define AC_MSG_LENGTH		10

extern uint8_t* AC_Stop_msg(void);
extern uint8_t* AC_Forward_msg(void);
extern uint8_t* AC_Reverse_msg(void);

extern uint8_t* AC_Freq_msg(uint16_t	_freq);
extern uint8_t* AC_Accel_msg(uint16_t	_accel);
extern uint8_t* AC_Decel_msg(uint16_t _decel);


uint8_t*  make_AC_Msg(uint8_t	sync1, uint8_t	sync2,
										uint8_t	add_high, uint8_t	add_low,
										uint8_t	val_high,uint8_t	val_low);

void  Calculate_CRC(uint8_t* inputs, int size);
