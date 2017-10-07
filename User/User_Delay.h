
/***********************************************************************/
#include "lpc17xx_libcfg.h"

/************************** PRIVATE VARIABLES *************************/
/* SysTick Counter */
volatile unsigned long SysTickCnt;


/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************/

/*********************************************************************//**
 * @brief	SysTick interrupt handler sub-routine
 * @param[in]	None
 * @return 	None
 **********************************************************************/
void SysTick_Handler (void)
{
  SysTickCnt++;
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************/

/*********************************************************************//**
 * @brief	Delay Function
 * @param[in]	tick	number of System ticks that will be Delayed
 * @return	None
 **********************************************************************/
void Delay (unsigned long tick)
{
  unsigned long systickcnt;

  systickcnt = SysTickCnt;
  while ((SysTickCnt - systickcnt) < tick);
}

/*********************************************************************//**
 * @brief	Return System Time in System Tick Counter
 * @param[in]	None
 * @return	None
 **********************************************************************/
unsigned long SystemTime(void)
{
  return SysTickCnt;
}
