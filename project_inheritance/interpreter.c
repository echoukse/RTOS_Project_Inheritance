// UARTIntsTestMain.c
// Runs on LM4F120/TM4C123
// Tests the UART0 to implement bidirectional data transfer to and from a
// computer running HyperTerminal.  This time, interrupts and FIFOs
// are used.
// Daniel Valvano
// September 12, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Program 5.11 Section 5.6, Program 3.10

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

// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include "UART.h"
#include "ADC.h"
#include "ST7735.h"
#include "OS.h"

#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2
                              //    red, yellow,    green, light blue, blue, purple,   white,          dark
const int32_t COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}
void UserTask(void){
  static int i = 0;
  LEDS = COLORWHEEL[i&(WHEELSIZE-1)];
  i = i + 1;
}
//debug code
int main(void){
  char i;
  char string[20];  // global to assist in debugging
  uint32_t Command,ChannelNo;
	unsigned long Timerval;

  PLL_Init();               // set system clock to 50 MHz
  UART_Init();              // initialize UART
  OutCRLF();

	OS_AddPeriodicThread(&UserTask, 50000000, 7); // Added the User task periodically to timer0
  while(1){
    UART_OutString("Command: ");
		//1: ADC Read
		//2: LCD out
		//3: Timer Read
		//4: Timer clear
    
		Command=UART_InUDec();
		
		switch(Command){
			case 1: 
				UART_OutString("ADC Channel to read: ");
			  ChannelNo=UART_InUDec();
			  ADC_Open(ChannelNo);
			  UART_OutUDec(ADC_In());OutCRLF();
			  break;
			case 2:
				Output_Init();
				UART_OutString("Enter the string to print to LCD: ");
			  UART_InString(string,20);
				ST7735_Message (0, 1, string, 0);OutCRLF();
			  break;
			case 3:
				Timerval=OS_ReadPeriodicTime();
			  UART_OutString("Timer reads :");
			  UART_OutUDec(Timerval);OutCRLF();
        break;			
			case 4:
				Timerval=OS_ClearPeriodicTime();
			  UART_OutString("Timer cleared.");
			  UART_OutUDec(Timerval);OutCRLF();
			  break;
		}

  }
}
