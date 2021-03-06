// filename **********OS.H***********
// Real Time Operating System for Labs 2 and 3 
// Jonathan W. Valvano 1/27/14, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file
// You are required to implement the spirit of this OS

 
#ifndef __OS_H
#define __OS_H  1

#define NULL ( (void *) 0)
// edit these depending on your clock        
#define TIME_1MS    80000          
#define TIME_2MS    (2*TIME_1MS)  
#define TIME_500US  (TIME_1MS/2)  
#define TIME_250US  (TIME_1MS/5)  

#define NumThreads 10
#define StackSize 128
#define HG_Number 5 // Allow upto 5 HGs

#define MAX_FIFO_SIZE 256

extern unsigned long SystickRollover; 

typedef struct Sema4 Sema4Type;
typedef  struct  os_tcb TCBType;

/*Hourglass*/
typedef  struct  Hourglass HGType;
typedef HGType * HGPType;

typedef struct HGThreadlist{
	int TID;
	struct HGThreadlist *next;
}HGThreadlistType;

typedef struct HGlist{
	int HG_ID;
	struct HGlist *next;
}HGlistType;

struct os_tcb{		
	unsigned int *sp;
	TCBType *Next;
  TCBType *Prev;	
	unsigned long ID;
	unsigned long sleep;
	unsigned long blocked;
	unsigned long actual_priority;
	unsigned long priority;
	Sema4Type *Semaptr; //List of semaphores held by this thread
	HGlistType *HGptr; //List of hourglass constructs on which task is blocked
};



/*Hourglass*/
struct  Hourglass{
	int HG_ID;
	HGThreadlistType *InsideList; //List of the threads inside the CS of this hourglass
	TCBType *WaitNonExclusiveList;  //head of the blocked list on this hourglass - list of readers
	TCBType *WaitBarrierList; //head of threads waiting for the CS to empty. But they dont want to get into the CS
	TCBType *WaitExclusiveList; //head of threads waiting for the CS to empty, wanting to get into the CS alone
	int free; //Are there any threads in the CS
	int ExclusiveRunning; //Exclusive thread is inside the critical section
	int Elevated; //Elevated priority
	int priority; //Priority of the lowest thread inside the CS
	struct Hourglass *next_HG; // so that we can make a list of all HGs held by a Thread	
};

struct Events{
	int ID;
	unsigned long time;
};

typedef struct Events EventType;

extern EventType myevents[100];
extern int curr_event;

extern long MaxJitterper[2];
// feel free to change the type of semaphore, there are lots of good solutions
struct  Sema4{
  long Value;   // >0 means free, otherwise means busy        
// add other components here, if necessary to implement blocking
	TCBType *BlockedList;  //head of the blocked list on this sema4
  TCBType *curr_holder; //the TCB which holds the sema4
  struct Sema4 *next_sema4; // list to all sema4 held by task holding this sema4	
};

extern unsigned long MAIL;
extern  Sema4Type boxfree;
extern  Sema4Type datavalid;
extern unsigned long FIFO[MAX_FIFO_SIZE];
extern unsigned long *FIFO_start;
extern unsigned long *FIFO_end;
extern  Sema4Type FIFO_mutex;
extern  Sema4Type FIFO_data_avail;
extern  Sema4Type FIFO_free;
// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void); 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value); 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt); 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt); 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt); 

// ******** OS_Block_bWait ************
// block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_Block_bWait(Sema4Type *semaPt); 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt); 

// ******** OS_bSignal ************
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_Block_bSignal(Sema4Type *semaPt); 

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
   unsigned long stackSize, unsigned long priority);


int OS_AddThread_with_HGBarrier(void(*task)(), unsigned long stackSize, unsigned long priority, HGType *lock);

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread_param(void(*task)(unsigned long), 
   unsigned long stackSize, unsigned long priority, unsigned long);

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void);

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
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
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority,int num);

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
int OS_AddSW1Task(void(*task)(void), unsigned long priority);

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
int OS_AddSW2Task(void(*task)(void), unsigned long priority);

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime); 

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void); 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void);
 
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size);

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data);  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void);

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void);

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void);

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data);

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void);

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void);

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
long OS_TimeDifference(unsigned long start, unsigned long stop);

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void);

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void);

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice);

/*Hourglass*/
void OS_HGInit( HGType *lock, int val);

/*Hourglass NOT AN API*/
//Put the thread in the InsideList if blocked = 0, 
//else if the priority is greater than HG priority, let it go in the inside list
//if the priority of this thread is lower, put it in NonExclusiveWaitList
void OS_HGreg( HGType *lock, int TID);

/*Hourglass NOT AN API*/
//Get back the priority it should have, based on the other locks it holds
//if this was the last thread in CS, 
//wake either the threads in NonExclusive list with higher priorities than writer/barrier, or
//Exclusivewaitlist if there is anybody
//All the threads in Barrier list
void OS_HGDereg( HGType *lock);

/*Hourglass*/
//This is a wait_exclusive
//sets the HG->block to 1 and makes this thread wait till the HG->free is 1. 
//No new thread with a lower priority will be allowed to register till this thread leaves the CS.
//If HG is already blocked, puts this thread in WaitExclusiveList
void OS_HGWait( HGType *lock); 


/*Hourglass*/
//Called when we dont care how many threads are inside CS.
//Called by readers . If no blocked, go into CS, Insidelist
//Else, if lower priority, go into the NonExclusiveWaitlist
void OS_HGWaitNonExclusive( HGType *lock); 


/*Hourglass*/
//Get back the priority it should have, based on the other locks it holds
//if this was the last thread in CS, 
//wake either the threads in NonExclusive list with higher priorities than writer/barrier, or
//Exclusivewaitlist if there is anybody
//All the threads in Barrier list
void OS_HGSignal( HGType *lock); 

/*Hourglass*/
//Calls signal implicitly.
//Just makes the thread wait in the WaitBarrierList till the CS is empty. 
//Once the CS is empty, the thread is let go. Shifted to RunList.
void OS_HGSyncThreads( HGType *lock); 

void Jitter_Calculate(int num);

void Jitter(void);   // prints jitter information (write this)

#endif

