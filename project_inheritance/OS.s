
  EXPORT  OSCtxSw
  EXPORT  PendSV_Handler
  EXPORT  LaunchOS


  PRESERVE8
  THUMB
  AREA CODE, CODE, READONLY
  IMPORT RunPt
  IMPORT NextRunPt
	  
OSCtxSw
    LDR     R0, =0xE000ED04                                  ; Trigger the PendSV exception (causes context switch)
    LDR     R1, =0x10000000
    STR     R1, [R0]
    BX      LR

PendSV_Handler ; 1) R0-R3,R12,LR,PC,PSR on MSP
  MRS    R3, PRIMASK  
  CPSID I ; 2) Prevent interrupt during switch
  PUSH {R4-R11} ; 3) Save remaining regs r4-11
  LDR R0, =RunPt ; 4) R0=pointer to RunPt, old thread
  LDR R1, [R0] ; R1 = RunPt
  STR SP, [R1] ; 5) Save MSP into TCB
  LDR R1, =NextRunPt ; 6) R1 = RunPt->next
  LDR R1, [R1]
  STR R1, [R0] ; RunPt = R1
  LDR SP, [R1] ; 7) new thread SP in R2
  POP {R4-R11}
  ;ORR LR, LR, #0x04 ; 0xFFFFFFFD (return to thread PSP)
  MSR PRIMASK, R3 ; 9) run with interrupts enabled
  BX LR ; 10) restore R0-R3,R12,LR,PC,PSR

LaunchOS
  LDR R0, =RunPt
  LDR R2,[R0]
  LDR SP,[R2]
  POP {R4-R11}
  POP {R0-R3}
  POP {R12}
  POP {LR}  ;Discarded the original value
  POP {LR}  ;Loads PC
  POP {R1}  ;Discard PSR
  CPSIE I   ;Enable Interrupts
  BX LR
  
  ALIGN

  END
