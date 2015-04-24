// Timer0.c
// Runs on LM4F120/TM4C123
// Use Timer0 in 32-bit periodic mode to request interrupts at a
// particular period.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
  Program 7.5, example 7.6

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "OS.h"

#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08


#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable

#define TIMER_CFG_32_BIT_TIMER  0x00000000  // 32-bit timer configuration
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
                                            // Interrupt
#define TIMER_TAILR_M           0xFFFFFFFF  // GPTM Timer A Interval Load
                                            // Register

long Timer2Rollover,Timer4Rollover;
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask1)(void);   // user function
void (*PeriodicTask2)(void);   // user function
// ***************** Timer0_Init ****************
// Activate Timer0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
// NOTE: USING BOTH AND B TIMER0 SO USE ONLY 16BIT
void Timer0_Init(void(*task)(void), unsigned long period, unsigned long priority,int num){long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x01;  //clock to timer 0
  TIMER0_CFG_R = TIMER_CFG_16_BIT;  // 2) configure for 32-bit timer mode   
	if(num == 1){    //TIMER0B	
		TIMER0_CTL_R &= ~TIMER_CTL_TBEN; // 1) disable timer0B during setup
                                     //  configure for periodic mode, default down-count settings
		TIMER0_TBMR_R = TIMER_TBMR_TBMR_PERIOD;
		TIMER0_TBILR_R = period-1;       // 4) reload value
                                     // 5) clear timer0B timeout flag
		TIMER0_ICR_R |= TIMER_ICR_TBTOCINT;
		TIMER0_IMR_R |= TIMER_IMR_TBTOIM;// 6) arm timeout interrupt for Timer0B
		NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFFFF00)|(priority << 5); // 7) priority 
		NVIC_EN0_R |= NVIC_EN0_INT20;     // 8) enable interrupt 20 in NVIC  20 - Timer0B
		TIMER0_TBPR_R = 0;
		TIMER0_CTL_R |= TIMER_CTL_TBEN;  // 9) enable timer0A
		PeriodicTask1 = task;
	}

	if(num == 2){	
		TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer0A during setup
		
                                   // 3) configure for periodic mode, default down-count settings
		TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
		TIMER0_TAILR_R = period-1;       // 4) reload value
                                   // 5) clear timer0A timeout flag
		TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
		TIMER0_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
		NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|(priority << 29); // 7) priority 
		NVIC_EN0_R |= NVIC_EN0_INT19;     // 8) enable interrupt 19 in NVIC
		TIMER0_TAPR_R = 0;
		TIMER0_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer0A
		PeriodicTask2 = task; 
	} 	
  EndCritical(sr);
}

void Timer2_Init()
	{
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x04;
  TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer0A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER2_TAILR_R = 159999;       // 4) reload value
  //TIMER2_IMR_R &= ~TIMER_IMR_TATOIM; // 6) disable interrupts
	TIMER2_ICR_R = TIMER_ICR_TATOCINT;
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0x1FFFFFFF)| 0x00000000; // 7) priority  0 
  NVIC_EN0_R |= NVIC_EN0_INT23;     // 8) enable interrupt 23 in NVIC
  TIMER2_TAPR_R = 0;
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	Timer2Rollover = 0;
  EndCritical(sr);
}
	
void Timer4_Init()
	{
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x10; //##change
  TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer0A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER4_TAILR_R = 159999;       // 4) reload value
  //TIMER4_IMR_R &= ~TIMER_IMR_TATOIM; // 6) disable interrupts
	TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  TIMER4_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFF1FFFFF)| 0x00400000; // 7) priority 2  
  NVIC_EN2_R = 0x00000040;     // 8) enable interrupt 70 in NVIC  
  TIMER4_TAPR_R = 0;
  TIMER4_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	Timer4Rollover = 0;
  EndCritical(sr);
}
	
void Timer0B_Handler(void){
	//LEDS = GREEN;	
  TIMER0_ICR_R = TIMER_ICR_TBTOCINT;// acknowledge timer0B timeout
	Jitter_Calculate(1);
	if(curr_event<100){
	  myevents[curr_event].ID = -1;
	  myevents[curr_event++].time = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));
	}
	(*PeriodicTask1)();                // execute user task
	if(curr_event<100){
	  myevents[curr_event].ID = -2;
	  myevents[curr_event++].time = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));
	}
	//LEDS = ~GREEN;
}

void Timer0A_Handler(void){
	//LEDS = GREEN;	
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
	Jitter_Calculate(2);
	if(curr_event<100){
	  myevents[curr_event].ID = -3;
	  myevents[curr_event++].time = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));
  }
	(*PeriodicTask2)();                // execute user task
	if(curr_event<100){
		myevents[curr_event].ID = -4;
	  myevents[curr_event++].time = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));
	}
	//LEDS = ~GREEN;
}
void Timer2A_Handler(void){
	//LEDS = GREEN;	
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
  Timer2Rollover++;
	//LEDS = ~GREEN;
}

void Timer4A_Handler(void){
	//LEDS = GREEN;	
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer4A timeout
  Timer4Rollover++;
	//LEDS = ~GREEN;
}
