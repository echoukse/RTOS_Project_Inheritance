#ifndef __PortF_H__ // do not include more than once
#define __PortF_H__

void SW1_Init(unsigned long priority, void(*task)(void));
void SW2_Init(unsigned long priority, void(*task)(void));

#endif
