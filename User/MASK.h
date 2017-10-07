#include "stdint.h" 
//#define GETMASK(index, size) (((1 << (size)) - 1) << (index))
//#define READFROM(data, index, size) (((data) & GETMASK((index), (size))) >> (index))
//#define WRITETO(data, index, size, value) ((data) = ((data) & (~GETMASK((index), (size)))) | ((value) << (index)))

//uint8_t setMask(uint8_t data, uint8_t index)
//{
//    data = data | (255 & (1 << index));
//    return data;
//}

//uint8_t getMask(uint8_t data, uint8_t index)
//{
//   uint8_t mask = (data >> index) & 1;
//   return mask;
//}

#define setMask(data, index) (data | (255 & (1 << index)))
#define getMask(data, index) ((data >> index) & 1)
