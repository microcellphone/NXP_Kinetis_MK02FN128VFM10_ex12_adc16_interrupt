#include "my_delay.h"

volatile uint32_t SysTickCnt;

void SysTick_Handler (void)
{
  SysTickCnt++;
}

void SysTick_Delay (uint32_t tick)
{
	uint32_t systickcnt;

  systickcnt = SysTickCnt;
  while ((SysTickCnt - systickcnt) < tick);
}
