#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "PortF.h"


void (*SW1Task)(void);   // user function
void (*SW2Task)(void);   // user function

// initialize edge trigger on SW1
void SW1_Init(unsigned long priority, void(*task)(void)){ 
  SW1Task = task;	
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)| priority << 21; //priority included
	NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

// initialize edge trigger on SW2
void SW2_Init(unsigned long priority, void(*task)(void)){                          
	SW2Task = task;	
	SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  	//unlock the lock register
  GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	//set commit register
  GPIO_PORTF_CR_R = 0xFF;
	GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF0 input (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF0   
  GPIO_PORTF_PCTL_R &= ~0x0000000F; // configure PF0 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x01;    //     PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x01;    //     PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x01;      // (e) clear flag0 ##check
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)| priority << 21; //priority included    ##TODO PF4 and PF0 cause same interrupt PRIORITY???
	NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  
}

//handler for portF
void GPIOPortF_Handler(void){
  if((GPIO_PORTF_MIS_R & 0x01) == 0x01) // PF0 caused the interrupt
	 {
		 GPIO_PORTF_ICR_R = 0x01;      // acknowledge flag0
	   //LEDS &=~GREEN;
	   (*SW1Task)();
	 }
	else if ((GPIO_PORTF_MIS_R & 0x010) == 0x10) // PF4 caaused the interrupt
	 {
		 GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
	   //LEDS &=~GREEN;
	   (*SW1Task)();
	 }	 
}
