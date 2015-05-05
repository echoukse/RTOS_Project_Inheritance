// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3 Testmain4  and main
// Lab3: Testmain5 Testmain6, Testmain7, and main (with SW2)

// Jonathan W. Valvano 1/31/14, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task, samples PD3
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PD3 Ain3 sampled at 2k, sequencer 3, by DAS software start in ISR
// PD2 Ain5 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include "OS.h"
#include "inc/tm4c123gh6pm.h"
#include "ST7735.h"
#include "ADC.h"
#include "UART.h"
#include <string.h> 
#include "PLL.h"
#include "Timer0.h"

//Sema4Type LCD_free;

unsigned long NumCreated;   // number of foreground threads created
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork = 0;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
#define FS 400            // producer/consumer sampling
#define RUNLENGTH (20*FS) // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US // DAS 2kHz sampling period in system time units

//---------------------User debugging-----------------------
unsigned long jitter;                    // time between measured and expected, in us
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
long diff;
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define LEDS      (*((volatile uint32_t *)0x40025038))

#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

void PortE_Init(void){ unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x10;       // activate port E
  delay = SYSCTL_RCGC2_R;        
  delay = SYSCTL_RCGC2_R;         
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port F
  while((SYSCTL_PRGPIO_R&0x0020) == 0){};// ready?
  GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x0E;     // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x0E;        // enable digital I/O on PF3-1
                                   // configure PF3-1 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF000F)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
  LEDS = 0;		// turn all LEDs off
}

void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}



HGType lock;
int count = 0;
void HGT1(void){
	while(1){
		OS_HGWait(&lock);
		count++;
		LEDS = BLUE;
		for(int i =0 ; i<100000; i++){};
		OS_HGSignal(&lock);
	}
	OS_Kill();	
}
int count2 = 0;
void HGT2(void){
	while(1){
		OS_HGWait(&lock);
		count2++;
		LEDS = RED;
		for(int i =0 ; i<100000; i++){};
		OS_HGSignal(&lock);
	}
	OS_Kill();	
}


int HGTEST1main(void){
	PortF_Init();
	OS_Init();	
	Timer2_Init();
  Timer4_Init();
	OS_HGInit(&lock, 1);
	OS_AddThread(&HGT1, 128, 1);
	OS_AddThread(&HGT2, 128, 1);
	OS_Launch(TIME_1MS);
}






HGType mutex1, mutex2;
 void thread_p1(void){  // foreground thread - highest priority       
  for(;;){
		OS_HGWait(&mutex1); // gets blocked the first time control reaches here 
    OS_HGWait(&mutex2); 
		LEDS |= GREEN;
		for(int i=0;i<50;++i){};
		OS_HGSignal(&mutex2);
   	OS_HGSignal(&mutex1);
		LEDS &= ~GREEN;	
  }
}
 
void thread_p2(void){  // foreground thread        
  for(;;){
		LEDS |= BLUE;
		for(int i=0;i<2;++i){};
		LEDS &=~BLUE;	
		OS_Kill();	
  }
}

void thread_p0(void){  // foreground thread        
  for(;;){
		LEDS |= BLUE;
		for(int i=0;i<2;++i){};
		LEDS &=~BLUE;	
		OS_Kill();	
  }
}

void thread_p3(void){  // foreground thread        
  for(;;){
		OS_HGWait(&mutex2);   // acquire mutex2 
    OS_HGSignal(&mutex1);  //signal mutex 1 to wake up thread_p1
		LEDS |= RED;
		NumCreated += OS_AddThread(&thread_p2,128,2); // now control goes to thread_p2 - priority inversion
		NumCreated += OS_AddThread(&thread_p0,128,0); 
    for(int i=0;i<3;++i){};		//duration priority is inverted     
		LEDS &= ~RED;
		OS_HGSignal(&mutex2);		
  }
}
 

int HGTEST2main(void){ 
  OS_Init();           // initialize, disable interrupts
	UART_Init();
	PortE_Init();
  PortF_Init();
	//Output_Init();  //##uncomment
	Timer2_Init();
  Timer4_Init();

	OS_HGInit(&mutex1,0); //mutex is initialized to 0 to create the situation for inversion.
  OS_HGInit(&mutex2,1);

  NumCreated = 0 ;
	
 
	NumCreated += OS_AddThread(&thread_p1,128,1);
	//NumCreated += OS_AddThread(&thread_p2,128,2);
  NumCreated += OS_AddThread(&thread_p3,128,3); 
	//NumCreated += OS_AddThread(&thread_p4,128,4);

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


int countr1 =0;
int countr2 =0;
int countr3 =0;
int countr4 =0;
HGType ReadHG;
void reader1(void){
	while(1){
		OS_HGWaitNonExclusive(&ReadHG);
		countr1++;
		LEDS = RED;
		OS_Sleep(3);
		OS_HGSignal(&ReadHG);
		OS_Sleep(3);
	}
}

void reader2(void){
	while(1){
		OS_HGWaitNonExclusive(&ReadHG);
		countr2++;
		LEDS = BLUE;
		OS_Sleep(2);
		OS_HGSignal(&ReadHG);
		OS_Sleep(2);
	}
}
void reader3(void){
	while(1){
		OS_HGWaitNonExclusive(&ReadHG);
		countr3++;
		LEDS = GREEN;
		OS_Sleep(1);
		OS_HGSignal(&ReadHG);
	}
}
void reader4(void){
	while(1){
		OS_HGWaitNonExclusive(&ReadHG);
		countr4++;
		LEDS = RED+BLUE;
		OS_HGSignal(&ReadHG);
	}
}
void writer(void){
	while(1){
		OS_Sleep(10);
		OS_HGWait(&ReadHG);
		countr4++;
		LEDS = GREEN+BLUE;
		OS_Sleep(2);
		OS_HGSignal(&ReadHG);
	}
}

int HGTest3main(void){
	OS_Init();  
  PortF_Init();
	Timer2_Init();
  Timer4_Init();

	OS_HGInit(&ReadHG,1);

  NumCreated = 0 ;
	
 
	NumCreated += OS_AddThread(&reader1,128,1);
	NumCreated += OS_AddThread(&reader2,128,2);
  NumCreated += OS_AddThread(&reader3,128,3); 
  NumCreated += OS_AddThread(&reader4,128,4);
	NumCreated += OS_AddThread(&writer,128,0);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//Domino elevation of priority
int testcount4_1 =0;
int testcount4_2 =0;
int testcount4_3 =0;
int testcount4_4 =0;
HGType test4_HG1,test4_HG2,test4_HG3;
void test4thread_1(void){
	while(1){
		OS_Sleep(8);
		OS_HGWait(&test4_HG3);
		
		testcount4_1++;
		LEDS = RED;
		OS_Sleep(3);
		OS_HGSignal(&test4_HG3);
		OS_Sleep(3);
	}
}

void test4thread_2(void){ //waiting for thread3
	while(1){
		OS_Sleep(4);
		OS_HGWait(&test4_HG3);
		OS_HGWait(&test4_HG2);
		testcount4_2++;
		LEDS = BLUE;
		OS_Sleep(2);
		OS_HGSignal(&test4_HG2);
		OS_HGSignal(&test4_HG3);
		OS_Sleep(2);
	}
}
void test4thread_3(void){  //waiting for thread4
	while(1){
		OS_Sleep(1);
		OS_HGWait(&test4_HG2);
		OS_HGWait(&test4_HG1);
		testcount4_3++;
		LEDS = GREEN;
		OS_Sleep(1);
		OS_HGSignal(&test4_HG1);
		OS_HGSignal(&test4_HG2);
	}
}
void test4thread_4(void){
	while(1){
		OS_HGWait(&test4_HG1);
		OS_Sleep(10); 
		testcount4_4++;
		LEDS = RED+BLUE;
		OS_HGSignal(&test4_HG1);
	  }
  }


int HGtestmain4(void){
	OS_Init();  
  PortF_Init();
	Timer2_Init();
  Timer4_Init();

	OS_HGInit(&test4_HG1,1);
  OS_HGInit(&test4_HG2,1);
	OS_HGInit(&test4_HG3,1);
  
  NumCreated = 0 ;
	 
	NumCreated += OS_AddThread(&test4thread_4,128,4);
	NumCreated += OS_AddThread(&test4thread_3,128,3);
  NumCreated += OS_AddThread(&test4thread_2,128,2); 
  NumCreated += OS_AddThread(&test4thread_1,128,1);
  OS_Launch(TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//TEST 5: Multiple writers
// adjust sleep timing such that
// writer 2 - holds the HG
// writer 1,writer 3,reader 1,reader 2 are waiting

int testcount5_1 =0;
int testcount5_2 =0;
int testcount5_3 =0;
int testcount5_4 =0;
int testcount5_5 =0;
HGType test5_HG1;
void test5reader_1(void){
	while(1){
		OS_Sleep(4);
		OS_HGWaitNonExclusive(&test5_HG1);
		OS_Sleep(4);
		testcount5_1++;
		LEDS = RED;
		OS_Sleep(3);
		OS_HGSignal(&test5_HG1);
	}
}

void test5reader_2(void){ 
	while(1){
		OS_Sleep(4);
		OS_HGWaitNonExclusive(&test5_HG1);
		OS_Sleep(4);
		testcount5_2++;
		LEDS = BLUE;
		OS_Sleep(2);
		OS_HGSignal(&test5_HG1);
	}
}

void test5writer_1(void){
	while(1){
		OS_Sleep(10);
		OS_HGWait(&test5_HG1); 
		testcount5_3++;
		LEDS = RED+BLUE;
		OS_HGSignal(&test5_HG1);
	  }
  }

void test5writer_2(void){  //
	while(1){
		OS_HGWait(&test5_HG1);
		OS_Sleep(5);
		testcount5_4++;
		LEDS = GREEN;
		OS_Sleep(1);
		OS_HGSignal(&test5_HG1);
	}
}

void test5writer_3(void){
	while(1){
		OS_HGWait(&test5_HG1); 
		testcount5_5++;
		LEDS = RED+BLUE;
		OS_HGSignal(&test5_HG1);
	  }
  }
int main(void){
	OS_Init();  
  PortF_Init();
	Timer2_Init();
  Timer4_Init();

	OS_HGInit(&test5_HG1,1);
	
  NumCreated = 0 ;
	 
	NumCreated += OS_AddThread(&test5reader_1,128,3);
	NumCreated += OS_AddThread(&test5reader_2,128,3);
  NumCreated += OS_AddThread(&test5writer_1,128,1); 
  NumCreated += OS_AddThread(&test5writer_2,128,1);
	NumCreated += OS_AddThread(&test5writer_3,128,1);
  OS_Launch(TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


//barrier
//thread 1,2,3 barrier one
//thread 3,4,5 barrier 2
// thread 6 independent thread 
int testcount6_1 =0;
int testcount6_2 =0;
int testcount6_3 =0;
int testcount6_4 =0;
int testcount6_5 =0;
int testcount6_6 =0;

HGType test6_HGbarrier1,test6_HGbarrier2;

void test6thread_1(void){
	while(1){
		testcount6_1++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		LEDS = RED;
		OS_Sleep(5);
		testcount6_1++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		OS_Sleep(3);
	}
}

void test6thread_2(void){ //waiting for thread3
	while(1){
		testcount6_2++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		LEDS = BLUE;
		OS_Sleep(5);
		testcount6_2++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		OS_Sleep(3);
	}
}
void test6thread_3(void){  //waiting for thread4
	while(1){
		testcount6_3++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		LEDS = RED+BLUE;
		OS_Sleep(5);
		testcount6_3++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		OS_Sleep(3);
	}
}
void test6thread_4(void){
	while(1){
		testcount6_4++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		LEDS = GREEN;
		OS_Sleep(5);
		testcount6_4++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		OS_Sleep(3);
	  }
  }

	void test6thread_5(void){  //waiting for thread4
	while(1){
		testcount6_5++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		LEDS = GREEN+RED;
		OS_Sleep(5);
		testcount6_5++;
		OS_HGSyncThreads(&test6_HGbarrier1);
		OS_Sleep(3);
	}
}
void test6thread_6(void){
	while(1){
		testcount6_6++;
		LEDS = RED+BLUE+GREEN;
	  }
  }

int HGTest6main(void){
	OS_Init();  
  PortF_Init();
	Timer2_Init();
  Timer4_Init();

	OS_HGInit(&test6_HGbarrier1,0);
  OS_HGInit(&test6_HGbarrier2,0);
  
  NumCreated = 0 ;
	 
	NumCreated += OS_AddThread_with_HGBarrier(&test6thread_1,128,1,&test6_HGbarrier1); //ID = 2
	NumCreated += OS_AddThread_with_HGBarrier(&test6thread_2,128,3,&test6_HGbarrier1); //ID = 3
  NumCreated += OS_AddThread_with_HGBarrier(&test6thread_3,128,4,&test6_HGbarrier1); //ID = 4
  NumCreated += OS_AddThread_with_HGBarrier(&test6thread_4,128,5,&test6_HGbarrier1); //ID = 5
	NumCreated += OS_AddThread_with_HGBarrier(&test6thread_5,128,6,&test6_HGbarrier1); //ID = 6
  NumCreated += OS_AddThread(&test6thread_6,128,2);
  OS_Launch(TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
