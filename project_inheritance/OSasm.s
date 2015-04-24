
  EXPORT  OSCtxSw
  EXPORT  PendSV_Handler
  EXPORT  LaunchOS
 ; EXPORT  SysTick_Handler
 ; EXPORT  HardFault_Handler


  PRESERVE8
  THUMB
  AREA CODE, CODE, READONLY
  IMPORT OldPt
  IMPORT RunPt
  IMPORT NewPt
	  
OSCtxSw
    LDR     R0, =0xE000ED04                                  ; Trigger the PendSV exception (causes context switch)
    LDR     R1, =0x10000000
    STR     R1, [R0]
    ;BX      LR
	POP {R0-R3}
	POP {R12}
	POP {LR}
	POP {PC}


PendSV_Handler
  CPSID I ; 2) Prevent interrupt during switch
  PUSH {R4-R11} ; 3) Save remaining regs r4-11
  LDR R0, =OldPt ; 4) R0=pointer to old thread
  CMP R0, #0x0  ; is_kill
  LDRNE R1, [R0] ; R1 = OldPt
  STRNE SP, [R1] ; 5) Save MSP into TCB
  LDR R1, =NewPt ; 6) R1 = RunPt->next
  LDR R1, [R1]
  LDR R0, =RunPt
  STR R1, [R0] ; RunPt = R1
  LDR SP, [R1] ; 7) new thread SP in R2
  POP {R4-R11}
 ORR LR, LR, #0x09 ; 0xFFFFFFF9 (return to thread MSP)
 CPSIE I ; 9) run with interrupts enabled
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
  
;HardFault_Handler
 ; tst LR, #4                                                
 ; ite eq                                                   
 ; mrseq r0, msp                                            
 ; mrsne r0, psp                                            
 ; ldr r1, [r0, #24]                                         
 ; ldr r2, handler2_address_const                           
 ; bx r2                                                    
 ; handler2_address_const: .word prvGetRegistersFromStack   
    
  ALIGN

  END
