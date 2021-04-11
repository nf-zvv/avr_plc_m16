
;------------------------------------------------------------------------------
; PWM Init
;------------------------------------------------------------------------------
PWM_INIT:
			; Timer 1, OCR1A, OCR1B
			; Fast PWM, 8-bit (Mode 5)
			; WGM13 = 0,WGM12 = 1,WGM11 = 0,WGM10 = 1
			; Set OC1A on compare match, clear OC1A at BOTTOM (inverting mode)
			InReg	r16,TCCR1A
			ori		r16,(1 << COM1A1)|(1 << COM1A0)|(1 << COM1B1)|(1 << COM1B0)|(1 << WGM10)
			OutReg	TCCR1A,r16
			; Prescaler 256
			ldi		r16,(1 << WGM12)|(1 << CS12)|(0 << CS11)|(0 << CS10)
			OutReg	TCCR1B,r16
			; Set default value
			ldi		r16,255
			cli
			OutReg	OCR1AH,r16
			OutReg	OCR1AL,r16
			OutReg	OCR1BH,r16
			OutReg	OCR1BL,r16
			sei
			
			; Timer 2
			; Fast PWM (Mode 3)
			; WGM21 = 1, WGM20 = 1
			;  Set OC2 on compare match, clear OC2 at BOTTOM, (inverting mode)
			; COM21 = 1, COM20 = 1
			; Prescaler 256
			; CS22 = 1, CS21 = 1, CS20 = 0
			InReg	r16,TCCR2
			ori		r16,(1 << COM21)|(1 << COM20)|(1 << WGM21)|(1 << WGM20)|(1 << CS22)|(1 << CS21)
			OutReg	TCCR2,r16
			; Set default value
			ldi		r16,255
			OutReg	OCR2,r16
			ret


;------------------------------------------------------------------------------
; PWM0 - Timer 1, OCR1A
; IN: r16 [0..255]
;------------------------------------------------------------------------------
PWM0_SET:
			cli
			OutReg	OCR1AH,__zero_reg__
			OutReg	OCR1AL,r16
			sei
			ret

;------------------------------------------------------------------------------
; PWM1 - Timer 1, OCR1B
; IN: r16 [0..255]
;------------------------------------------------------------------------------
PWM1_SET:
			cli
			OutReg	OCR1BH,__zero_reg__
			OutReg	OCR1BL,r16
			sei
			ret


;------------------------------------------------------------------------------
; PWM2 - Timer 2, OCR2
; IN: r16 [0..255]
;------------------------------------------------------------------------------
PWM2_SET:
			OutReg	OCR2,r16
			ret


PWM0_GET:
			InReg	r16,OCR1AL
			ret
