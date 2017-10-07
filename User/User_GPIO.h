/***********************************************************************//**
 * @file		User_GPIO.h
 * @brief		Contains all macro definitions and function prototypes
 * 				support for GPIO firmware library on LPC17xx
 * @version		1.00
 * @date		1. January. 2017
 * @author		ElecSys Co.
 **************************************************************************
 * Software that is described herein is Peripheral Layer2 Functions for GPIO
 **************************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup LAYER2 GPIO
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef USER_GPIO_H_
#define USER_GPIO_H_

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_gpio.h"

/* Public Macros -------------------------------------------------------------- */
/** @defgroup LAYER2_GPIO_Public_Macros
 * @{
 */

#define ILockD      {0,0,0}
#define IUnlockD    {0,1,0}
#define IRelayD     {0,3,0}
#define ITest1D     {0,2,0}

#define OECUD       {3,7,1}
#define OTrunkD     {2,0,1}
#define OTest1D     {3,7,1}
#define OTest2D     {1,1,1}
#define OTest3D     {1,2,1}
#define OTest4D     {1,3,1}


#define OECU_On         pinOn(&OECU)              // Lset off
#define OECU_Off        pinOff(&OECU)             // Lset on 
#define OECU_Toggle     pinToggle(&OECU)          // Lset Toggle
#define OTrunk_On       pinOff(&OTrunk)            // Lstart off
#define OTrunk_Off      pinOn(&OTrunk)           // Lstart on 
#define OTrunk_Toggle   pinToggle(&OTrunk)        // Lstart Toggle
#define OTest1_On       pinOff(&OTest1)            // Lkick off
#define OTest1_Off      pinOn(&OTest1)           // Lkick on 
#define OTest1_Toggle   pinToggle(&OTest1)        // Lkick set Toggle
#define OTest2_On       pinOff(&OTest2)            // Lkick off
#define OTest2_Off      pinOn(&OTest2)           // Lkick on 
#define OTest2_Toggle   pinToggle(&OTest2)        // Lkick set Toggle
#define OTest3_On       pinOff(&OTest3)            // Lkick off
#define OTest3_Off      pinOn(&OTest3)           // Lkick on 
#define OTest3_Toggle   pinToggle(&OTest3)        // Lkick set Toggle
#define OTest4_On       pinOff(&OTest4)            // Lkick off
#define OTest4_Off      pinOn(&OTest4)           // Lkick on 
#define OTest4_Toggle   pinToggle(&OTest4)        // Lkick set Toggle

#define ILock_h         pinStatus(&ILock)         // Sset status
#define IUnlock_h       pinStatus(&IUnlock)       // Sdown status
#define IRelay_h        pinStatus(&IRelay)        // Sup status
#define ITest1_h        pinStatus(&ITest1)        // Sup status


typedef struct __pin_struct
{
  int port;
  int position;
  int direction;
}pin;

void pinConfig(pin *p);
void pinOn(pin *p);
void pinOff(pin *p);
void pinToggle(pin *p);
void pinFlash(pin *p , unsigned long period, int repeat, char mode, char end);
void pinSet(pin *p,int s);
int pinStatus(pin *p);
uint8_t GPIO_IntCheck(pin *p, uint8_t polarity);
void GPIOINT_Init(pin *p, uint8_t intNum,uint8_t priority, uint8_t polarity);

#endif /* USER_GPIO_H_ */
