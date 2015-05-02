// OS.c
// Runs on LM4F120/TM4C123
//Esha Choukse & Kishore Punniyamurthy

#include <stdint.h>
#include <stdio.h>
#include "Timer0.h"
#include "SysTickInts.h"
#include "PLL.h"
#include "inc/tm4c123gh6pm.h"
#include "OS.h"
#include "PortF.h"
#include "UART.h"

#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

extern void LaunchOS(void);
extern void OSCtxSw(void);
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);
extern int StartCritical(void);
extern void EndCritical(int);
extern void WaitForInterrupt(void);

TCBType *RunPt, *RunHead; //Pointer to current thread running and the ready list
TCBType *SleepPt; //Pointer to the sleep list
TCBType *CurrSleepPt; 
TCBType *OldPt,*NewPt, *KillPt;

unsigned int is_sleep = 0, is_kill = 0;
unsigned long SystickRollover;
unsigned int Stack[NumThreads+1][StackSize];

TCBType TCBs[NumThreads+1];

unsigned long unique_IDs[NumThreads+1];

EventType myevents[100];
int curr_event;

// globals
unsigned long MAIL = 0;
Sema4Type boxfree;
Sema4Type datavalid;
unsigned long FIFO[MAX_FIFO_SIZE];
unsigned long *FIFO_start;
unsigned long *FIFO_end;
Sema4Type FIFO_mutex;
Sema4Type FIFO_data_avail;
Sema4Type FIFO_free;

Sema4Type mutex1;
Sema4Type mutex2;

int OS_Launched=0;
int Current_thread_count = 0;

TCBType * InsertTask(TCBType *head, TCBType *Node){
	TCBType *temp;
	if(head==NULL){
		head = Node;
		Node->Next = Node;
		Node->Prev = Node;
		return head;
	}
	temp = head->Prev;
	
	temp->Next = Node;
	Node->Prev = temp;
	Node->Next = head;
	head->Prev = Node;
	return head;
}

//Dummy thread at lowest priority to avoid OS crash
//TODO fix so that it is actually lowest priority
void Dummy(void){
	while(1){
	}
}

long MaxJitterper[2] = {0,0};             // largest time jitter between interrupts in usec
#define JITTERSIZE 64

unsigned long const JitterSize1=JITTERSIZE;
unsigned long const JitterSize2=JITTERSIZE;
unsigned long JitterHistogram1[JITTERSIZE]={0,};
unsigned long JitterHistogram2[JITTERSIZE]={0,};
unsigned long Jitterloop[2] = {0,0};
unsigned long PERIOD[2];

void Jitter_Calculate(int num){
	unsigned static long LastTime[2];  // time at previous ADC sample
  unsigned long thisTime[2];         // time at current ADC sample
	long diff[2];
	unsigned long jitter[2];                    // time between measured and expected, in us
	
	if(num==1){
		thisTime[0] = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));       // current time, 12.5 ns
    Jitterloop[0]++;        // calculation finished
    if(Jitterloop[0]>1 && (LastTime[0]<thisTime[0])){    // ignore timing of first interrupt
		  diff[0] = OS_TimeDifference(LastTime[0],thisTime[0]);
      if(diff[0]>PERIOD[0]){
        jitter[0] = (diff[0]-PERIOD[0]+4)/8;  // in 0.1 usec
      }else{
        jitter[0] = (PERIOD[0]-diff[0]+4)/8;  // in 0.1 usec
      }
      if(jitter[0] > MaxJitterper[0]){
        MaxJitterper[0] = jitter[0]; // in 0.1 usec
      }       // jitter should be 0
      if(jitter[0] >= JitterSize1){
        jitter[0] = JITTERSIZE-1;
      }
      JitterHistogram1[jitter[0]]++;
		}
    LastTime[0] = thisTime[0];
		
	}
	if(num==2){
		
		thisTime[1] = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));       // current time, 12.5 ns
    Jitterloop[1]++;        // calculation finished
    if(Jitterloop[1]>1 && (LastTime[1]<thisTime[1])){    // ignore timing of first interrupt
		  diff[1] = OS_TimeDifference(LastTime[1],thisTime[1]);
			
      if(diff[1]>PERIOD[1]){
        jitter[1] = (diff[1]-PERIOD[1]+4)/8;  // in 0.1 usec
      }else{
        jitter[1] = (PERIOD[1]-diff[1]+4)/8;  // in 0.1 usec
      }
      if(jitter[1] > MaxJitterper[1]){
        MaxJitterper[1] = jitter[1]; // in 0.1 usec
      }       // jitter should be 0
      if(jitter[1] >= JitterSize2){
        jitter[1] = JITTERSIZE-1;
      }
      JitterHistogram2[jitter[1]]++;
		}
    LastTime[1] = thisTime[1];
	}
	
}

void Jitter(void){
  UART_OutString("Jitter 1 ="); UART_OutUDec(MaxJitterper[0]);
	UART_OutChar(CR);
  UART_OutChar(LF);
  UART_OutString(", Jitter 2 ="); UART_OutUDec(MaxJitterper[1]);
	UART_OutChar(CR);
  UART_OutChar(LF);
  
}
//insert HG structure in the list in ascending order of priority (lowest first)
HGlistType * InsertHGlist(HGlistType *head, HGlistType *Node){
	HGlistType *temp, *temp1;
	if(head==NULL){ //first thread to enter the critical section 
		head = Node;
		Node->next = NULL;
                //head->free = 0;
		return head;
	}
	temp1 = temp = head;
//	while(temp->next)
//	  temp = temp->next;
//	temp->next = Node;
//	Node->next = NULL;
//	return head;
   
        while((TCBs[temp->TID].priority > TCBs[Node->TID].priority) && (temp->next)) 
	{         
                temp1 = temp;
		temp = temp->next;
	}	
	
	if(TCBs[temp->TID].priority <= TCBs[Node->TID].priority) //last TCB in the list, Add Node between temp and head
	  {
	        	temp1->next = Node;
                        Node->next = temp;
	  }	
        
        temp->next = Node;
        Node->next = NULL; 

}

HGlistType* DeleteHGlistType(HGlistType *head, int TID){
	HGlistType *temp,temp1;
	if(TID == head->TID)
	 {
 		 head = head->next;
		 return head;
	 }
	temp = head;
	while((temp->next->TID !=TID)&&(temp!=NULL))
		 temp = temp->next;
	if(temp->next->TID == TID) //match found
          {
	     temp1 = temp->next;
             temp->next = temp->next->next;
             free(temp1);     
          }
  return head;	
}

Sema4Type * InsertSema4(Sema4Type *head, Sema4Type *Node){
	Sema4Type *temp;
	if(head==NULL){
		head = Node;
		Node->next_sema4 = NULL;
		return head;
	}
	temp = head;
	while(!temp->next_sema4)
	  temp = temp->next_sema4;
	temp->next_sema4 = Node;
	Node->next_sema4 = NULL;
	return head;
}

Sema4Type* DeleteSema4(Sema4Type *head, Sema4Type *Node){
	Sema4Type *temp;
	if(Node == head)
	 {
 		 head = head->next_sema4;
		 return head;
	 }
	temp = head;
	while((temp->next_sema4 !=Node)&&(temp!=NULL))
		 temp = temp->next_sema4;
	if(temp->next_sema4 == Node) //match found
	  temp->next_sema4 = temp->next_sema4->next_sema4;
  return head;	
}


TCBType * Insert_priorityTask(TCBType *head, TCBType *Node){
	TCBType *temp_tcbptr1;
	if(head==NULL){
		head = Node;
		Node->Next = Node;
		Node->Prev = Node;
		return head;
	}
		
	temp_tcbptr1 = head;
	
	// Go till the priority level is reached, or list ends
	while((temp_tcbptr1->priority <= Node->priority) && (temp_tcbptr1->Next != head)) 
	{
		temp_tcbptr1 = temp_tcbptr1->Next;
	}	
	
	if(temp_tcbptr1->priority <= Node->priority) //last TCB in the list, Add Node between temp and head
	  {
		  temp_tcbptr1->Next = Node;
			Node->Next = head;
			head->Prev = Node;
			Node->Prev = temp_tcbptr1;
		}	 
		else //Node is to be inserted before temp, since temp is of a lower priority level
		{
 			temp_tcbptr1->Prev->Next = Node;
			Node->Next = temp_tcbptr1;
			Node->Prev = temp_tcbptr1->Prev;
			temp_tcbptr1->Prev = Node;
		}

	if(head->priority <= Node->priority)
		   return head;
		else
			 return Node;	
	
}

int is_in_list(TCBType *head, TCBType *Node){
	TCBType *temp;
	temp = head;
	do{
		if(temp->ID==Node->ID)
			return 1;
		temp = temp->Next;		
	}while(temp->ID != head->ID);
	return 0;
}

TCBType* DeleteTask(TCBType *head, TCBType *Node, int iskill, int *done){
	if(!is_in_list(head, Node)){
		*done = 0;
		return head;
	}
	if(iskill)
		unique_IDs[Node->ID] = 0;
	if(Node->Prev->ID == Node->ID){
		head = NULL;
		return head;
	}
	Node->Prev->Next = Node->Next;
	Node->Next->Prev = Node->Prev;
	if(head->ID == Node->ID){
		head = Node->Next;
	}
  return head;	
}

unsigned long findID(){
	for(int i=1; i<=NumThreads; i++){
		if(unique_IDs[i]==0){
			unique_IDs[i]=1;
			return i;
		}
	}
	return 0;
}

TCBType * Change_priority(TCBType *head, HGType * lock, int new_priority ){
         	int done =1;
                HGlistType temp = lock->head; 
                while(temp) // iterate over all threads currently inside CS
                 {
                   if(TCBs[temp->TID].priority > new_priority)
                  { 
		  do{
		        head = DeleteTask(RunHead, &TCBs[temp->TID], 0,&done);
		    }while(done==0);
		   TCBs[temp->TID].priority =  new_priority;      // priority inheritance;  
	           head=Insert_priorityTask(head, &TCBs[temp->TID]);
                  }
                 }
        }

void initTCB(TCBType *myTCB, unsigned long myID, unsigned long priority){
	myTCB->ID = myID;
	myTCB->priority = priority;
	myTCB->actual_priority = priority;
	myTCB->blocked = 0;
	myTCB->sleep = 0;
	myTCB->sp = &Stack[myID][StackSize - 16];
	Stack[myID][StackSize-1] = 0x01000000; // Thumb bit
	//Stack[myID][StackSize-2] = (unsigned int) task; // PC	
	Stack[myID][StackSize-3] = 0xFFFFFFF9; // R14
	Stack[myID][StackSize-4] = 0x12121212; // R12
	Stack[myID][StackSize-5] = 0x03030303; // R3
	Stack[myID][StackSize-6] = 0x02020202; // R2
	Stack[myID][StackSize-7] = 0x01010101; // R1
	Stack[myID][StackSize-8] = 0x00000000; // R0
	Stack[myID][StackSize-9] = 0x11111111; // R11
	Stack[myID][StackSize-10] = 0x10101010;
	Stack[myID][StackSize-11] = 0x09090909;
	Stack[myID][StackSize-12] = 0x08080808;
	Stack[myID][StackSize-13] = 0x07070707;
	Stack[myID][StackSize-14] = 0x06060606;
	Stack[myID][StackSize-15] = 0x05050505;
	Stack[myID][StackSize-16] = 0x04040404; //R4
}

// Subroutine to wait 1 msec
// Inputs: None
// Outputs: None
// Notes: ...
void Delay1millisec(uint32_t n){uint32_t volatile time;
  while(n){
    time = 72724*2/91;  // 1msec, tuned at 80 MHz
    while(time){
	  	time--;
    }
    n--;
  }
}

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void){
	DisableInterrupts();
	PLL_Init();
	RunPt = NULL;
	RunHead = NULL;
	SleepPt = NULL;
	Current_thread_count = 0;
	curr_event = 0;
	for(int i=0; i<=NumThreads; i++)
	  unique_IDs[i] = 0;
	//TODO : add a dummy thread if current run list is NULL
	OS_AddThread(&Dummy,128,7);
	NVIC_ST_CTRL_R = 0; //Disable SysTick
	NVIC_ST_CURRENT_R = 0;
	NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0xFF1FFFFF) | 0x00E00000; //P level 7 for PendSV
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	semaPt->Value = value;
	semaPt->BlockedList = NULL;
}

// ******** OS_Block_Sleep ************
// place this thread into a dormant state, called in blocking semaphore wait()
// input:  semaphore on which the task is blocked
// output: none

void OS_Block_Sleep(Sema4Type * Semaptr){	
	long sr = StartCritical(); 
	int done = 1;
	do{
	RunHead = DeleteTask(RunHead, RunPt, 0, &done);
	}while(done==0);
	//Semaptr->BlockedList = InsertTask(Semaptr->BlockedList, temp);   // for bounded waiting
  Semaptr->BlockedList = Insert_priorityTask(Semaptr->BlockedList, RunPt);	//for priority
	EndCritical(sr);
	OS_Suspend();	
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
	DisableInterrupts();
	while((semaPt->Value) <= 0){
		EnableInterrupts();
		//OS_Suspend();
		DisableInterrupts();
		}
	(semaPt->Value) = (semaPt->Value) - 1;
	EnableInterrupts();
}

// ******** OS_Block_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Block_Wait(Sema4Type *semaPt){
	long sr = StartCritical(); 
	(semaPt->Value) = (semaPt->Value) - 1;
	if((semaPt->Value) < 0){
		//OS_Block_Sleep(semaPt);	//call block_sleep
		int done = 1;
		do{
		RunHead = DeleteTask(RunHead, RunPt, 0, &done);
		}while(done==0);
		//Semaptr->BlockedList = InsertTask(Semaptr->BlockedList, temp);   // for bounded waiting
		semaPt->BlockedList = Insert_priorityTask(semaPt->BlockedList, RunPt);	//for priority
    EndCritical(sr);
		OS_Suspend();
		}
	else
		EndCritical(sr);
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
	long status;
	status = StartCritical();
	(semaPt->Value) = (semaPt->Value) + 1;
	EndCritical(status);
}

// ******** OS_Block_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Block_Signal(Sema4Type *semaPt){
	long status;
	status = StartCritical();
	TCBType *temp_TCBptr;
	temp_TCBptr = semaPt->BlockedList; //this TCB task (highest priority (or head) is woken up
	(semaPt->Value) = (semaPt->Value) + 1;
	if((semaPt->Value) <= 0){
		  int done =1;
		  do{
		    semaPt->BlockedList = DeleteTask(semaPt->BlockedList, semaPt->BlockedList, 0, &done);
			}while(done==0);
			RunHead=Insert_priorityTask(RunHead, temp_TCBptr);
			EndCritical(status);
			OS_Suspend();	
		}
	else
			EndCritical(status);
	
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
	DisableInterrupts();
	while((semaPt->Value) <= 0){
		EnableInterrupts();
		DisableInterrupts();
		}
	(semaPt->Value) = 0;
	EnableInterrupts();
}

// ******** OS_Block_bWait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
// ESSENTIALLY SAME AS OS_Block_Wait
void OS_Block_bWait(Sema4Type *semaPt){  
	long sr = StartCritical();
  int done=1;	
	(semaPt->Value) = (semaPt->Value) - 1;
	if((semaPt->Value) < 0){  //the task is blocked
		
		if(semaPt->curr_holder->priority > RunPt->priority) //if current sema4 holder has lower priority than blocked task
	 	  {
				// the priority of the currernt holder of the sema4 is changed
				// done by deleting the current thread holding sema4 from run list
        // changing the priority
        // Add the same task back with updated priority				
				done =1;
		    do{
		        RunHead = DeleteTask(RunHead, semaPt->curr_holder, 0,&done);
		      }while(done==0);
		        semaPt->curr_holder->priority =  RunPt->priority;      // priority inheritance;  
		        RunHead=Insert_priorityTask(RunHead, semaPt->curr_holder);
		  }		
    
		// regular stuff - add running thread to blocked list 			
	  done =1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
		}while(done==0);
		//Semaptr->BlockedList = InsertTask(Semaptr->BlockedList, temp);   // for bounded waiting
		semaPt->BlockedList = Insert_priorityTask(semaPt->BlockedList, RunPt);	//for priority 		
    EndCritical(sr);
		OS_Suspend();
		// when control reaches here the task gets the sema4 - unblocked
		// make it the current sema4 holder  
		semaPt->curr_holder = RunPt;
		RunPt->Semaptr = InsertSema4(RunPt->Semaptr,semaPt);
		// a task acquiring a sema4 on being unblocked means there is no task waiting on the same sema4 with higher priority so 
		// NO NEED to check and increase priority.
		}
	else
	 {
		semaPt->curr_holder = RunPt;
    RunPt->Semaptr = InsertSema4(RunPt->Semaptr,semaPt);		 
		EndCritical(sr);   
	 } 
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
	(semaPt->Value) = 1;
}

// ******** OS_Block_bSignal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
// ESSENTIALLY SAME AS OS_Block_SIGNAL

void OS_Block_bSignal(Sema4Type *semaPt){
	long status;
	Sema4Type *sema4_iter;
	int highest_priority = RunPt->actual_priority;
	status = StartCritical();
	TCBType *temp_TCBptr;
	temp_TCBptr = semaPt->BlockedList; //this TCB task (highest priority (or head) is woken up
	(semaPt->Value) = (semaPt->Value) + 1;
	
	//## added priority inheritance
	RunPt->Semaptr = DeleteSema4(RunPt->Semaptr,semaPt); //delete the sema4 from the list of sema4s held by this thread
	sema4_iter = RunPt->Semaptr;
	// routine to restore the priority
	// priority = original priority or blocked thread priority whichever is higher
	while(!sema4_iter)  //if tasks holds other sema4s //ASK: What is this? why is it !sema4?
	{
		if(!sema4_iter->BlockedList) //if sema4 has other tasks blocked on it
		{	
     if( highest_priority > sema4_iter->BlockedList->priority ) //only work if Blocked list is according to priority 
         highest_priority = sema4_iter->BlockedList->priority;
	  }     
     sema4_iter = sema4_iter->next_sema4;
	}
	
	//updating the priority
	// done by deleting the current thread holding sema4 from run list
  // changing the priority
  // Add the same task back with updated priority		 
	int done =1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
		}while(done==0);
		RunPt->priority = highest_priority;  // priority inheritance;  
		RunHead=Insert_priorityTask(RunHead, RunPt);
	
	//regular stuff - wake up blocked thread	
	if((semaPt->Value) <= 0){
		  done = 1;
		  do{
		    semaPt->BlockedList = DeleteTask(semaPt->BlockedList, temp_TCBptr, 0, &done);
			}while(done==0);
			RunHead = Insert_priorityTask(RunHead, temp_TCBptr); //has to be priority
			EndCritical(status);
			OS_Suspend();	
		}
	else
	 {
			EndCritical(status);
		  OS_Suspend(); //priority has changed so context switch
	 }	
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(), unsigned long stackSize, unsigned long priority){
	long sr=StartCritical();	
	unsigned long myID = findID();
	if(myID==0){
		EndCritical(sr);
		return 0;
	}
	TCBType *myTCB;
	myTCB = &TCBs[myID];
	initTCB(myTCB, myID, priority);
	Stack[myID][StackSize-2] = (unsigned int) task; // PC
	RunHead= Insert_priorityTask(RunHead, myTCB);
	Current_thread_count++;
	if(OS_Launched)
	{
		EndCritical(sr);
		OS_Suspend();
	}		
	else
	  EndCritical(sr);
	return 1;
}

//******** OS_AddThread_param *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread_param(void(*task)(unsigned long), unsigned long stackSize, unsigned long priority, unsigned long data){
	long sr=StartCritical();	
	unsigned long myID = findID();
	if(myID==0){
		EndCritical(sr);
		return 0;
	}
	TCBType *myTCB;
	myTCB = &TCBs[myID];
	initTCB(myTCB, myID, priority);
	Stack[myID][StackSize-2] = (unsigned int)(task); // PC
	Stack[myID][StackSize-8] = data; // R0
	RunHead= Insert_priorityTask(RunHead, myTCB);
	Current_thread_count++;
	if(OS_Launched)
	{
		EndCritical(sr);
		OS_Suspend();
	}		
	else
	  EndCritical(sr);
	return 1;
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){
  return RunPt->ID;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
//				 num - to specify the number of periodic task 1 or 2 							
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority, int num){
	if(num==1)
		PERIOD[0] = period;
	else
		PERIOD[1] = period;
	Timer0_Init(task, period, priority,num); // initialize timer0 (16 Hz)
  return 1;
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
	SW1_Init(priority, task);
	return 1;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
	SW2_Init(priority, task);
	return 1;
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){	
	long sr = StartCritical();
	if(sleepTime!=0){
		CurrSleepPt = RunPt;
		is_sleep =1;
		RunPt->sleep = sleepTime;
		int done = 1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0, &done);
		}while(done==0);
		SleepPt=Insert_priorityTask(SleepPt, RunPt);
	}
	EndCritical(sr);
	OS_Suspend();	
}


// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	DisableInterrupts();
	int done = 1;
	do{
		if(done==0){
			EnableInterrupts();
			OS_Suspend();
		}
		DisableInterrupts();
		RunHead = DeleteTask(RunHead, RunPt, 1, &done);
	}while(done==0);
	Current_thread_count--;
	is_kill = 1;
	NVIC_ST_CURRENT_R = 0; // Clear Count
	NVIC_INT_CTRL_R = 0x04000000; // Trigger Systick
	EnableInterrupts();
	while(1){}
}	

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){	
	long sr = StartCritical();
	NVIC_ST_CURRENT_R = 0; // Clear Count
	NVIC_INT_CTRL_R = 0x04000000; // Trigger Systick
	EndCritical(sr);
	//Delay1millisec(5);
}
 
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size){
 while(size>256) {}; 	// limbo to show size exceeds 
 OS_InitSemaphore(&FIFO_mutex,1);
 OS_InitSemaphore(&FIFO_free,(size));
 OS_InitSemaphore(&FIFO_data_avail,0);	 
 for(int i=0;i<size;++i)
   {
      FIFO[i] = 0;
	 }	 	
	FIFO_start = FIFO_end = FIFO; 
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	 int ret= 0;

   //OS_Wait(&FIFO_free);
	 //int sr = StartCritical();
	 //if(FIFO_free.Value<=0){
	 //EndCritical(sr);
	 //ret = 0;		 
	 //}
	 //else{
	   //EndCritical(sr);
		 OS_Block_Wait(&FIFO_free);
  	 OS_Block_bWait(&FIFO_mutex); 	
	   *FIFO_end = data;
	   if (FIFO_end == &FIFO[255]) 
			 FIFO_end = &FIFO[0];  // circular FIFO - roll over
		 else
			 FIFO_end++;		
     OS_Block_bSignal(&FIFO_mutex);
     OS_Block_Signal(&FIFO_data_avail);
		 ret =1;
	 //}
   return ret;				 				 
}  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
	unsigned long data;
	 OS_Block_Wait(&FIFO_data_avail);
	 OS_Block_bWait(&FIFO_mutex); 	
	 data = *FIFO_start;
	 if (FIFO_start == &FIFO[255]) 
				   FIFO_start = &FIFO[0];  // circular FIFO - roll over
				 else
					 FIFO_start++;		
  
	 OS_Block_bSignal(&FIFO_mutex);
   OS_Block_Signal(&FIFO_free);
	 return data;			 
			
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
   return FIFO_free.Value;  // based on usage may require mutex
}

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(){
 OS_InitSemaphore(&boxfree,1);
 OS_InitSemaphore(&datavalid,0);	
}
// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
	OS_Block_bWait(&boxfree);
	MAIL = data;
	OS_Block_bSignal(&datavalid);
}
// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
	unsigned long data;
	OS_Block_bWait(&datavalid);
	data = MAIL;
	OS_Block_bSignal(&boxfree);
	return data;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	return TIMER2_TAR_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
long OS_TimeDifference(unsigned long start, unsigned long stop){
   return (stop-start);
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  TIMER4_TAV_R = 0;
  Timer4Rollover=0;	
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){	
	return (((Timer4Rollover*160000)+(160000-TIMER4_TAR_R))*12.5)/1000000;
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice){
	SystickRollover = 0;
	SysTick_Init(theTimeSlice);
	NVIC_ST_CTRL_R = 0x7; //Enable Core Clock and interrupt armed
	RunPt = RunHead;
	OS_Launched = 1;
	LaunchOS();
}

/*Hourglass*/
void OS_HGInit( HGType *lock, int val){
	lock->free = val;
	lock->head = NULL;
}

/*Hourglass*/
void OS_HGreg( HGType *lock){
	// This thread just registers, so no priority inheritance done here.
	//First get a new Node initialisedi
        int TID = RunPt->ID;
	HGlistType *newnode ;
	newnode = (HGlistType*)malloc(sizeof(HGlistType));
	newnode->TID = TID;
	//Add it to the list of registered ones // TODO correct registering
        lock->head = InsertHGlist(lock->head,newnode); 
//	if(lock->head == NULL)
//		lock->free = 0;
//	newnode->next = lock->head;
//	lock->head = newnode;
}

//TODO: put start and end criticals!!!!!
/*Hourglass*/
void OS_HGDereg( HGType *lock){
	//This thread needs to roll back the priority to original, or max of other sema4s/HGs
	
	long status = StartCritical();
	int TID = RunPt->ID; 
	//First remove it from the list
        lock->head = DeleteHGlistType(lock->head,TID); 
//	HGlistType *temp1, *temp2;
//	temp1 = lock->head;
//	if(temp1==NULL)
//	  return;
//	temp2 = temp1->next;
//	if(temp1->TID == TID)
//		lock->head = temp2;
//	while(temp2!=NULL){
//		if(temp2->TID == TID){
//			temp1->next = temp2->next;
//			break;
//		}
//		temp1 = temp2;
//		temp2 = temp2->next;
//	}

	//Now rollback its priority
	//## added priority inheritance
//      Sema4Type *sema4_iter;
	int highest_priority = RunPt->actual_priority;
	RunPt->HGptr = DeleteHG(RunPt->HGptr,lock); //delete the HG  from the list of HGs held by this thread
        if(RunPt->priority == lock->priority)
	{
          lock->priority = TCBs[lock->head->TID].priority;
        }      
//	sema4_iter = RunPt->Semaptr;
//	// routine to restore the priority
//	// priority = original priority or blocked thread priority whichever is higher
//	while(sema4_iter)  //if tasks holds other sema4s //ASK: What is this? why is it !sema4?
//	{
//		if(sema4_iter->BlockedList) //if sema4 has other tasks blocked on it
//		{	
//    	if( highest_priority > sema4_iter->BlockedList->priority ) //only work if Blocked list is according to priority 
//         highest_priority = sema4_iter->BlockedList->priority;
//	  }     
//     	sema4_iter = sema4_iter->next_sema4;
//	}
        
        TCBType *temp_TCBptr;
	HGType *HG_iter = RunPt->HGptr;
	while(HG_iter)  //if tasks holds other HGs //ASK: What is this? why is it !sema4?
	{
	   // if(HG_iter->priority != -1) //if HGs has other tasks blocked on it
	   // {	
     	  if( highest_priority > HG_iter->priority ) //only work if Blocked list is according to priority 
            highest_priority = HG_iter->priority;
	  //  }     
     	HG_iter = HG_iter->next_HG;
	}
	
	//updating the priority
	// done by deleting the current thread holding sema4 from run list
        // changing the priority
        // Add the same task back with updated priority
        int done =1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
		}while(done==0);
		RunPt->priority = highest_priority;  // priority inheritance;  
		RunHead=Insert_priorityTask(RunHead, RunPt);
        
		
		if(lock->head == NULL){
			lock->free = 1;
                        lock->block = 0;
			//regular stuff - wake up blocked thread	
    	        //	if(lock->blocked != 0){
	//			TCBType *Woken_TCBptr;
	//			done = 1;
	//			do{
	//				lock->WaitEmptyList = DeleteTask(lock->WaitEmptyList, Woken_TCBptr, 0, &done);
	//				}while(done==0);
	//			//Last blocker woken, update the priority of HG
	//			if(lock->WaitEmptyList == NULL){
	//				lock->priority = -1;
	//			}
	//			else{
	//				lock->priority = lock->WaitEmptyList->priority; // Next blocking thread's priority
	//			}
	//			RunHead = Insert_priorityTask(RunHead, Woken_TCBptr); //Insert the woken thread 
	//			EndCritical(status);
	//			OS_Suspend();	
		//	}
		//	else{
		//		EndCritical(status);
		//		OS_Suspend();
		//	}
              if(lock->WaitExclusiveList && lock->WaitEmptyList) //if both exclusive and non-sxclusive threads are blocked
              {  
              if(lock->WaitExclusiveList->priority <= lock->WaitEmptyList->priority)
                {                         
	          temp_TCBptr = lock->WaitExclusiveList; 
                  //wake highest priority exclusive thread
                   done = 1;
		  do{
		    lock->WaitExclusiveList = DeleteTask(lock->WaitExclusiveList, temp_TCBptr, 0, &done);
			}while(done==0);
			RunHead = Insert_priorityTask(RunHead, temp_TCBptr); //has to be priority
			EndCritical(status);
			OS_Suspend();	
		}
               else   //wake from EmptyList
                {                  
	          temp_TCBptr = lock->WaitEmptyList; 
                  //wake all non-exclusive threads which have higher priority than the head of exclusive thread
                   while(temp_TCBptr && (lock->WaitExclusiveList->priority >= lock->WaitEmptyList->priority)) {
                   done = 1;
		  do{
		    lock->WaitEmptyList = DeleteTask(lock->WaitEmptyList, temp_TCBptr, 0, &done);
			}while(done==0);
			RunHead = Insert_priorityTask(RunHead, temp_TCBptr); //has to be priority
                     temp_TCBptr = lock->WaitEmptyList;
                    } 
			EndCritical(status);
			OS_Suspend();
                }
	       }
               else if (lock->WaitExclusiveList->priority) // only exclusive thread is blocked
                  {
                     temp_TCBptr = lock->WaitExclusiveList; 
                  //wake highest priority exclusive thread
                   done = 1;
		  do{
		    lock->WaitExclusiveList = DeleteTask(lock->WaitExclusiveList, temp_TCBptr, 0, &done);
			}while(done==0);
			RunHead = Insert_priorityTask(RunHead, temp_TCBptr); //has to be priority
			EndCritical(status);
			OS_Suspend(); 
                  }
              else if(lock->WaitEmptyList->priority) //only non-exclusive threads are blocked
                {
		  temp_TCBptr = lock->WaitEmptyList; 
                  //wake all non-exclusive threads which have higher priority than the head of exclusive thread
                   while(temp_TCBptr && (lock->WaitExclusiveList->priority >= lock->WaitEmptyList->priority)) {
                   done = 1;
		  do{
		    lock->WaitEmptyList = DeleteTask(lock->WaitEmptyList, temp_TCBptr, 0, &done);
			}while(done==0);
			RunHead = Insert_priorityTask(RunHead, temp_TCBptr); //has to be priority
                     temp_TCBptr = lock->WaitEmptyList;
                    } 
			EndCritical(status);
			OS_Suspend();

                }  
	     }	
            else{ // still threads inside CS
			EndCritical(status);
		  OS_Suspend(); //priority has changed so context switch
	     }			
	
}

/*Hourglass*/
//sets the HG->block to 1 and makes this thread wait till the HG->free is 1. 
//No new thread will be allowed to register till this thread leaves the CS.
//If HG is already blocked, puts this thread in WaitList
//void OS_HGWait( HGType *lock){
//	long sr = StartCritical();
//        int TID = RunPt->ID; 
//	//Blocking happens without caring about whether there are readers in CS
//	lock->blocked = 1;
//        if(lock->priority < TCBs[TID].priority){  //check why <  , 
//		lock->priority = TCBs[TID].priority;
//	}
//	//If there are readers in the CS, get added to the waiting list
//	if(!lock->free){
//		int done =1;
//		do{
//		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
//		}while(done==0);
//		lock->WaitExclusiveList = Insert_priorityTask(lock->WaitExclusiveList, RunPt);		
//		EndCritical(sr);
//		OS_Suspend();
//	}
//	EndCritical(sr); //No Readers in the CS, go ahead
//}

void OS_HGWait( HGType *lock){
	long sr = StartCritical();
	int TID = RunPt->ID;
        lock->free = (lock->free) - 1;
	//If there are readers in the CS, get added to the waiting list
	if(lock->blocked){  //lock already acquired by an  exclusive task -> blocked =1
               //elevate priority if necessary
               if(lock->priority > RunPt->priority) //if registered thread has lower priority than blocked task  
                lock->priority = RunPt->priority;
                Change_priority(RunHead,lock,TCBs[TID].priority);
                //sequence to add the thread to exclusive blocked list 
                int done =1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
		}while(done==0);
		lock->WaitEmptyList = Insert_priorityTask(lock->WaitEmptyList, RunPt);		
		EndCritical(sr);
		OS_Suspend();
	}
        //else  // obtained lock in first go
        //{
          OS_HGreg(lock,TID);//the thread enters the CS so gets registered 
          lock->blocked = 0;
          if(lock->priority < TCBs[TID].priority)  
     	  lock->priority = TCBs[TID].priority;
	  EndCritical(sr); //No Readers in the CS, go ahead
        //}
}

void OS_HGWait_ex( HGType *lock){
	long sr = StartCritical();
	int TID = RunPt->ID;
        lock->free = (lock->free) - 1;
	//If there are readers in the CS, get added to the waiting list
	if(lock->free<0){  //lock already acquired
		
               //elevate priority if necessary
               if(lock->priority > RunPt->priority) //if registered threads holder has lower priority than blocked task 
                 lock->priority = RunPt->priority;
		 Change_priority(RunHead,lock,TCBs[TID].priority);
                //sequence to add the thread to exclusive blocked list 
                int done =1;
		do{
		  RunHead = DeleteTask(RunHead, RunPt, 0,&done);
		}while(done==0);
		lock->WaitExclusiveList = Insert_priorityTask(lock->WaitExclusiveList, RunPt);		
		EndCritical(sr);
		OS_Suspend();
	}
        //else  // obtained lock in first go
        // {
	  OS_HGreg(lock, TID);	
          lock->blocked = 1;   
     	  lock->priority = TCBs[TID].priority;
	  EndCritical(sr); //No Readers in the CS, go ahead
        //}

   
}


/*Hourglass*/
//If there are other threads in the WaitExclusiveList, puts one of those in CS
//Else sets the HG->blocked = 0
void OS_HGSignal( HGType *lock, int TID){
OS_HGDereg(lock);	
	
}

/*Hourglass*/
//Just makes the thread wait in the WaitEmptyList till the CS is empty. 
//Priority inheritance applies.
//Once the CS is empty, the thread is let go. Shifted to RunList.
void OS_HGBlock( HGType *lock, int TID){
	
	
}


//function to synct the threads
//Add the threads IDs in the HG.head 
// Set the HG.blocked  = 1
//during start , to do barriers
void OS_sync_threads(HGType *lock){

  int status = StartCritical();
  OS_HGDereg(lock);
  OS_Wait(lock);
  if(lock->WaitEmptyList == NULL) 
    lock->blocked = 1;
 EndCritical(status);
}
void SysTick_Handler(void){
	long sr= StartCritical();
	TCBType *temp, *next;
	SystickRollover++;
	KillPt = NULL;
	if(SleepPt!=NULL){
		temp = SleepPt;
		do{
			next = temp->Next;
			temp->sleep--;
			if(temp->sleep==0){
				int done =1;
				do{
				  SleepPt = DeleteTask(SleepPt, temp, 0, &done);
				}while(done==0);
				//##TODO check the Insert_priorityTask function
				RunHead=Insert_priorityTask(RunHead, temp);
				
			}
			temp = next;
		}while((SleepPt!=NULL) && (SleepPt->ID!=temp->ID));
	}
	if(is_kill){
		KillPt = RunPt;
		OldPt = 0;
		is_kill = 0;
		NewPt = RunHead;
	}
	//TODO: if is block
	
	else{
		if(!is_sleep && (RunHead->priority == RunPt->priority)  && is_in_list(RunHead,RunPt)){ //current thread is at highest priority, give chance to next thread in this priority
			if(RunPt->Next->priority == RunHead->priority){  // There is another thread at the same priority , give it chance
				OldPt = RunPt;  
				NewPt = RunPt->Next;
			}
			else{
				OldPt = RunPt;
				NewPt = RunHead;
			}
		}
		else{
			if(is_sleep)
				is_sleep = 0;
			OldPt = RunPt;
			NewPt = RunHead;
		}
	}
	if(curr_event<100){
	  myevents[curr_event].ID = NewPt->ID;
	  myevents[curr_event++].time = (Timer2Rollover*TIME_2MS+(TIME_2MS-OS_Time()));
	}
	NVIC_INT_CTRL_R = 0x10000000; // Trigger PendSV
	EndCritical(sr);
}
