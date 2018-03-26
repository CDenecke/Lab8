;***********************************************************
;*
;*	robot.asm
;*
;*	Receive data to be used for the TekBot
;*
;*	This is the RECEIVE skeleton file for Lab 8 of ECE 375
;*
;***********************************************************
;*
;*	 Author: Chase Denecke and Nickoli Londura
;*	   Date: March 7, 2018
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	waitcnt = r19			; Wait Loop Counter
.def	ilcnt = r20				; Inner Loop Counter
.def	olcnt = r21				; Outer Loop Counter
.def	movementReg = r22		; Register to hold movement command
.def	freezeCount = r23		; Counts how many times the robot has been frozen

.equ	WTime = 100				; Time to wait in wait loop

.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	BotAddress = $79		; The address corresponding to this particular bot

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////
.equ	MovFwd =  (1<<EngDirR|1<<EngDirL)	;0b01100000 Move Forward
.equ	MovBck =  $00						;0b00000000 Move Backward
.equ	TurnR =   (1<<EngDirL)				;0b01000000 Turn Right
.equ	TurnL =   (1<<EngDirR)				;0b00100000 Turn Left
.equ	Halt =    (1<<EngEnR|1<<EngEnL)		;0b10010000 Halt
.equ	Freeze = 0b01010101					;Freeze

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

.org	$0002
		rcall	HitRight		; Call hit right function
		reti
.org	$0004
		rcall	HitLeft			; Call hit left function
		reti
.org	$003C					; USART1, Rx complete interrupt
		rcall	USART_Receive	; Jump to USART_Receive
		reti
.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
		; Initialize the Stack Pointer
		ldi		mpr, low(RAMEND)
		out		SPL, mpr				; Load SPL with low byte of RAMEND
		ldi		mpr, high(RAMEND)
		out		SPH, mpr				; Load SPH with high byte of RAMEND

	
		
			; INT0 - INT1
		ldi		mpr, (0<<WskrR)|(0<<WskrL)
		out		DDRD, mpr				; Set the two least significant bits in PORTD to be inputs
		ldi		mpr, (1<<WskrR)|(1<<WskrL)
		out		PORTD, mpr				; Set Port D to input wiht Hi-Z
			; Port B
		ldi		mpr, (1<<EngEnL)|(1<<EngEnR)|(1<<EngDirR)|(1<<EngDirL)
		out		DDRB, mpr				; Set the most significant 4 bits in PORTB to be outputs
		ldi		mpr, $FF
		out		PORTB, mpr				; Initialize PORTB to 0

	;USART1
		;Set baudrate at 2400bps
		ldi		mpr, high(416)			; Load high byte of 0x01A0
		sts		UBRR1H, mpr				; UBRR1H in extended I/O space
		ldi		mpr, low(416)			; Load low byte of 0x01A0
		sts		UBRR1L, mpr				; UBRR1L in extended I/O space

		;Enable both transmitter and receiver, and receive interrupts
		ldi		mpr, (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1)
		sts		UCSR1B, mpr				; UCSR1B in extended I/O space

		;Set frame format: 8 data bits, 2 stop bits
		ldi		mpr, (0<<UMSEL1)|(1<<USBS1)|(1<<UCSZ11)|(1<< UCSZ10)
		sts		UCSR1C, mpr				; UCSR1C in extended I/O space

		;Initialize registers to 0
		clr		freezeCount				; Each bot starts out having been frozen 0 times
		ldi		movementReg, MovFwd		; Default movement is move forward
		ldi		waitcnt, 100			; Wait routine should wait for 100*10ms (1 second)

	;External Interrupts
		;Set the External Interrupt Mask
		ldi		mpr, (1<<INT0)|(1<<INT1); Enable HitRight and HitLeft interrupts
		out		EIMSK, mpr

		;Set the Interrupt Sense Control to falling edge detection
		ldi		mpr, (1<<ISC01)|(0<<ISC00)|(1<<ISC11)|(0<<ISC10)
		sts		EICRA, mpr				; Detect interrupts on falling edge

		; Turn on interrupts
		sei

	; Other

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
		out		PORTB, movementReg ;write the current movement to PortB in a loop
		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: HitRight
; Desc: Handles functionality of the TekBot when the right whisker
;		is triggered.
;-----------------------------------------------------------
HitRight:				
		cli							; Clear Global Interrupt
			
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					; Save mpr register

		;ldi		mpr, (0<<RXCIE1)	; Disable receive interrupt
		;sts		UCSR1B, mpr

		; Move Backwards for a second
		ldi		mpr, MovBck			; Load Move Backward command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	Wait				; Call wait function

		; Turn left for a second
		ldi		mpr, TurnL			; Load Turn Left Command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	Wait				; Call wait function

		ldi		mpr, (1<<INT0)|(1<<INT1)	; Set up to clear interrupt flag
		out		EIFR, mpr			; Clear the interrupt flag

		;ldi		mpr, (1<<RXCIE1)	; Enable receive interrupt
		;sts		UCSR1B, mpr

		pop		mpr					; Restore mpr register
		out		SREG, mpr			; Restore program state
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr register

		ret						; End a function with RET and enable interrupts

;-----------------------------------------------------------
; Func: HitLeft
; Desc: Handles functionality of the TekBot when the left whisker
;		is triggered.
;-----------------------------------------------------------
HitLeft:							
		cli							; Clear Global Interrupt

		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					; Save SREG register
		lds		mpr, UCSR1B
		push mpr

		ldi		mpr, (0<<RXCIE1)    ; Disable receive interrupt
		sts		UCSR1B, mpr

		; Move Backwards for a second
		ldi		mpr, MovBck			; Load Move Backward command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	Wait				; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR			; Load Turn Left Command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	Wait				; Call wait function

		ldi		mpr, (1<<INT0)|(1<<INT1)	; Set up to clear interrupt flag
		out		EIFR, mpr			; Clear the interrupt flag

		pop		mpr
		sts		UCSR1B, mpr			; Restore UCSR1B to enable recieve interrupts etc.
		pop		mpr					; Restore mpr register
		out		SREG, mpr			; Restore program state
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr register

		ret						; End a function with RET and enable interrupts

;-----------------------------------------------------------
; Func: Wait
; Desc: A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;-----------------------------------------------------------
Wait:							
		push	waitcnt				; Save wait register
		push	ilcnt				; Save ilcnt register
		push	olcnt				; Save olcnt register

Loop:	ldi		olcnt, 224			; load olcnt register
OLoop:	ldi		ilcnt, 237			; load ilcnt register
ILoop:	dec		ilcnt				; decrement ilcnt
		brne	ILoop				; Continue Inner Loop
		dec		olcnt				; decrement olcnt
		brne	OLoop				; Continue Outer Loop
		dec		waitcnt				; Decrement wait 
		brne	Loop				; Continue Wait loop	

		pop		olcnt				; Restore olcnt register
		pop		ilcnt				; Restore ilcnt register
		pop		waitcnt				; Restore wait register

		ret							; End a function with RET

;-----------------------------------------------------------
; Func: USART_Receive
; Desc: Receiving data
;-----------------------------------------------------------
USART_Receive:
		cli							; Clear Global Interrupt

		push	mpr					; Save mpr register

		lds		r17, UDR1			; Read data from receive data buffer
		;out PORTB, r17

		cpi		r17, BotAddress		; Compare the data with the bot address
		brne	EndAddressCompare
			set ;sets the T-flag in the status register
			rjmp	EndCompareAction
		EndAddressCompare:

		;Check to see if last packet received is freeze command
		cpi		r17, Freeze
		brne	EndFreezeCommand
			push	movementReg		;save the current movement command

			ldi		mpr, Halt		;halt the robot
			out		PORTB, mpr
			inc		freezeCount		;increment the freeze counter
			cpi		freezeCount, 3	;check to see if freeze counter is 3
			brne	NormalFreeze	
				FrozenForever:
				rjmp	FrozenForever;if freeze counter is 3, permanently stop the robot
			NormalFreeze:
			rcall	wait
			rcall	wait
			rcall	wait
			rcall	wait
			rcall	wait			;call wait 5 times to freeze the robot for 5 seconds

			pop		movementReg		;restore movement robot had before being frozen
			rjmp	EndCompareAction
		EndFreezeCommand:

		;If(last packet was address of this robot)
			;check to see what command is
			;execute it if it matches one of the listed commands
		brtc	EndCompareAction
			
			cpi		r17, 0b10110000; ($80|MovFwd>>1)	; Check if move forward command
			brne	EndMovFwd
				ldi		movementReg, MovFwd		
				clt										; Clear T flag in SREG
				rjmp	EndCompareAction
			EndMovFwd:

			cpi		r17, 0b10000000 ;($80|MovBck>>1)	; Check if move backward command
			brne	EndMovBck
				ldi		movementReg, MovBck
				clt
				rjmp	EndCompareAction	; Jump to the end of the interrupt
			EndMovBck:

			cpi		r17, ($80|TurnR>>1)		; Check if turn right command
			brne	EndTurnR
				ldi		movementReg, TurnR
				clt
				rjmp	EndCompareAction	; Jump to the end of the interrupt
			EndTurnR:

			cpi		r17, ($80|TurnL>>1)		; Check if turn left command
			brne	EndTurnL
				ldi		movementReg, TurnL
				clt
				rjmp	EndCompareAction	; Jump to the end of the interrupt
			EndTurnL:

			cpi		r17, ($80|Halt>>1)		; Check if halt command
			brne	EndHalt
				ldi		movementReg, Halt
				
				clt
				rjmp	EndCompareAction	; Jump to the end of the interrupt
			EndHalt:

			cpi		r17, 0b11111000			; Check if freeze command
			brne	EndCompareAction
				lds		mpr, UCSR1B			
				push mpr					; Save current interrupt settings

				ldi		mpr, (0<<RXCIE1)    ; Disable receive interrupt
				sts		UCSR1B, mpr

				ldi		r17, Freeze
				rcall	USART_Transmit		; Send the freeze signal
				clt

				pop		mpr
				sts		UCSR1B, mpr			; Restore UCSR1B to enable recieve interrupts etc.

		EndCompareAction:
		ldi		mpr, (1<<INT0)|(1<<INT1)	; Set up to clear interrupt flag
		out		EIFR, mpr			; Clear the interrupt flag							

		pop		mpr					; Restore mpr register
		
		ret							; End a function with RET

;-----------------------------------------------------------
; Func: USART_Transmit
; Desc: Transmitting data
;-----------------------------------------------------------
USART_Transmit:

		lds		mpr, UCSR1A
		sbrs	mpr, UDRE1				; Loop until UDR1 is empty
		rjmp	USART_Transmit
		sts		UDR1, r17				; Move data to transmit data buffer
		
		ret

;***********************************************************
;*	Stored Program Data
;***********************************************************

;***********************************************************
;*	Additional Program Includes
;***********************************************************

