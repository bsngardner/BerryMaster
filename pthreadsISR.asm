;	threadsISR.asm	12/01/2015
;*******************************************************************************
;  STATE        Task Control Block        Stacks (malloc'd)
;                          ______                                       T0 Stack
;  Running tcbs[0].thread | xxxx-+------------------------------------->|      |
;                 .stack  | xxxx-+-------------------------------       |      |
;                 .block  | 0000 |                               \      |      |
;                 .sleep  |_0000_|                       T1 Stack \     |      |
;  Ready   tcbs[1].thread | xxxx-+---------------------->|      |  \    |      |
;                 .stack  | xxxx-+------------------     |      |   \   |      |
;                 .block  | 0000 |                  \    |      |    -->|(exit)|
;                 .sleep  |_0000_|        T2 Stack   --->|r4-r15|       |------|
;  Blocked tcbs[2].thread | xxxx-+------->|      |       |  SR  |
;                 .stack  | xxxx-+---     |      |       |  PC  |
;                 .block  |(sem) |   \    |      |       |(exit)|
;                 .sleep  |_0000_|    --->|r4-r15|       |------|
;  Free    tcbs[3].thread | 0000 |        |  SR  |
;                 .stack  | ---- |        |  PC  |
;                 .block  | ---- |        |(exit)|
;                 .sleep  |_----_|        |------|
;
;*******************************************************************************

            .cdecls C,"msp430.h"            ; MSP430
            .cdecls C,"pthreads.h"			; threads header
            .cdecls C,"BerryMaster.h"

			;.def	TA_isr
			.ref	ctid
			.ref	tcbs
			;.ref	ERROR2

STACK_OVERFLOW_CHECK	.equ	1
_SYS					.equ	1
_ERR_STACK				.equ	3

tcb_thread	.equ	(tcbs + 0)
tcb_stack	.equ	(tcbs + 2)
tcb_block	.equ	(tcbs + 4)
tcb_sleep	.equ	(tcbs + 6)

; Code Section ------------------------------------------------------------

            .text                           ; beginning of executable code

; Timer A ISR -------------------------------------------------------------
TA_isr:     bic.w   #TAIFG|TAIE,&TA0CTL		; acknowledge & disable TA interrupt
			push	r15						; save current thread state
			push	r14
			push	r13
			push	r12
			push	r11
			push	r10
			push	r9
			push	r8
			push	r7
			push	r6
			push	r5
			push	r4
			mov.w	ctid,r4					; get current thread id
			add.w	r4,r4					; x sizeof(Tcb)
			add.w	r4,r4
			add.w	r4,r4
			mov.w	SP,tcb_stack(r4)		; save stack pointer

			.if STACK_OVERFLOW_CHECK
			cmp.w	SP,tcb_thread(r4)		; stack overflow?
			  jlo	TA_isr_02				; n
			;mov.w	#_SYS,r12				; y
			;mov.w	#_ERR_STACK,r13
			call	#handleError;#ERROR2	; ERROR2(_SYS, _ERR_STACK);
			.endif

TA_isr_02:	mov.w	#MAX_THREADS,r5			; limit search

TA_isr_04:	add.w	#1,ctid					; next thread
			and.w	#MAX_THREADS-1,ctid
			mov.w	ctid,r4					; get current thread id
			add.w	r4,r4					; x sizeof(Tcb)
			add.w	r4,r4
			add.w	r4,r4
			tst.w	tcb_thread(r4)			; thread?
			  jeq	TA_isr_06				; n
			tst.w	tcb_block(r4)			; y, blocked?
			  jeq	TA_isr_08				; n

TA_isr_06:	sub.w	#1,r5					; more?
			  jne	TA_isr_04				; y
			nop
			bis.w	#LPM0+GIE,SR			; n, goto sleep
			nop
			jmp		TA_isr_02

TA_isr_08:	mov.w	tcb_stack(r4),SP		; schedule thread
			pop		r4
			pop		r5
			pop		r6
			pop		r7
			pop		r8
			pop		r9
			pop		r10
			pop		r11
			pop		r12
			pop		r13
			pop		r14
			pop		r15
			bis.w	#TAIE,&TA0CTL			; enable TA interrupts
			reti

; Interrupt Vector --------------------------------------------------------
     ;       .sect   ".int44"                ; timer A section
     ;       .word   TA_isr                  ; timer A isr
            .end
