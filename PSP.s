
 ; PSP.s
 ;
 ;  Created on: Sep 25, 2022
 ;      Author: Sanket Hiremath


;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------

   .def enablePSPMode
   .def setPSP
   .def getPsp
   .def getMsp
   .def disablePrivileged
   .def enablePrivileged
   .def pushStack
   .def popStack
   .def pushPsp


;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------




;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

; Turns on the PSP mode(sets ASP bit to 1)
enablePSPMode:

  				MRS R1, CONTROL
    			ORR R1, #2
    			MSR CONTROL, R1
    			ISB
    			BX	LR


; Sets the start address for the PSP stack
setPSP:

    			MSR PSP,R0
    			BX  LR

; Gets the start address for the PSP stack
getPsp:
               MRS R0,PSP
               ISB
		       BX  LR

getMsp:
			   MRS R0,MSP
			   ISB
			   BX  LR

; Sets the TMPL bit to allow unprivileged software to run
disablePrivileged:
					MRS R1, control
					ORR R1, #1
					MSR CONTROL, R1
					ISB
					BX LR

; Clears the TMPL bit to allow only privileged software to run
enablePrivileged:
					MRS R1, control
					BIC R1, #1
					MSR CONTROL, R1
					ISB
					BX LR


pushStack:
				MRS  R0, PSP
				SUB  R0, #4
				STR  R11, [R0]
				SUB  R0, #4
				STR  R10, [R0]
				SUB  R0, #4
				STR  R9, [R0]
				SUB  R0, #4
				STR  R8, [R0]
				SUB  R0, #4
				STR  R7, [R0]
				SUB  R0, #4
				STR  R6, [R0]
				SUB  R0, #4
				STR  R5, [R0]
				SUB  R0, #4
				STR  R4, [R0]
				MSR  PSP, R0
				BX   LR

popStack:

				MRS  R0, PSP
				LDR  R4, [R0]
				ADD  R0, #4
				LDR  R5, [R0]
				ADD  R0, #4
				LDR  R6, [R0]
				ADD  R0, #4
				LDR  R7, [R0]
				ADD  R0, #4
				LDR  R8, [R0]
				ADD  R0, #4
				LDR  R9, [R0]
				ADD  R0, #4
				LDR  R10, [R0]
				ADD  R0, #4
				LDR  R11, [R0]
				ADD  R0, #4
				MSR  PSP, R0
				BX   LR

pushPsp:
				MRS R1, PSP
				SUB R1, #4
				STR R0, [R1]
				MSR PSP, R1
				BX LR

.endm

