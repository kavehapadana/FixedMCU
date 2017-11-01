
/* Includes ------------------------------------------------------------------- */
#include "User_GPIO.h"
#include "lpc17xx_pinsel.h"

void pinConfig(pin *p)
{
  pin *pin1;
  pin1=p;
  
  GPIO_SetDir(pin1->port, 1<<pin1->position, pin1->direction);
}

void pinOn(pin *p)
{
  pin *pin1;
  pin1=p;
  
  GPIO_SetValue(pin1->port, 1<<pin1->position);
}

void pinOff(pin *p)
{
  pin *pin1;
  pin1=p;
  
  GPIO_ClearValue(pin1->port, 1<<pin1->position);
}

void pinToggle(pin *p)
{
	pin *pin1;
  pin1=p;
  if(pinStatus(pin1))
		GPIO_ClearValue(pin1->port, (1<<pin1->position));
	else
		GPIO_SetValue(pin1->port, (1<<pin1->position));
}

void pinSet(pin *p,int s)
{
  int status;
  
  status=s&0x1;
  
  if(!status) pinOff(p);
  else pinOn(p); 
}

int pinStatus(pin *p)
{
  unsigned long status = 0;
	pin *pin1;
  pin1 = p;
  
  status = GPIO_ReadValue(pin1->port);
  status = (status>>(pin1->position))&0x01;
  return (int)status;
}

void GPIOINT_Init(pin *p, uint8_t intNum,uint8_t priority, uint8_t polarity)
{
  pin *pin1;
  
  pin1=p;
  
  NVIC_SetPriorityGrouping(4); //sets group priorities: 8 - subpriorities: 3
  switch (intNum)
  {
		case 0:
			NVIC_SetPriority(EINT0_IRQn, priority);  //000:10 (bit 7:3)  assign eint0 to group 0, sub-priority 2 within group 0
			break;
		case 1:
			NVIC_SetPriority(EINT1_IRQn, priority);  //000:10 (bit 7:3)  assign eint0 to group 0, sub-priority 2 within group 0
			break;
		case 2:
			NVIC_SetPriority(EINT2_IRQn, priority);  //000:10 (bit 7:3)  assign eint0 to group 0, sub-priority 2 within group 0
			break;
		case 3:
			NVIC_SetPriority(EINT3_IRQn, priority);  //000:10 (bit 7:3)  assign eint0 to group 0, sub-priority 2 within group 0
			break;
  }
  
  switch (intNum)
  {
		case 0:
			NVIC_EnableIRQ(EINT0_IRQn);
			break;
		case 1:
			NVIC_EnableIRQ(EINT1_IRQn);
			break;
		case 2:
			NVIC_EnableIRQ(EINT2_IRQn);
			break;
		case 3:
			NVIC_EnableIRQ(EINT3_IRQn);
			break;
  }
  
  GPIO_IntCmd(pin1->port, 1<<pin1->position, polarity);
}   

uint8_t GPIO_IntCheck(pin *p, uint8_t polarity)
{
  if (GPIO_GetIntStatus(p->port, p->position, polarity))
  {
    GPIO_ClearInt(p->port, (1<<p->position));
    return 1;
  }
  return 0;
}

