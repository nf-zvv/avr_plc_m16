#define F_CPU (16000000)

.nolist
.include "m16def.inc"
.include "macro.asm"
.include "eeprom_macro.asm"
.list
.listmac ; Enable expanding macros


; Нулевой регистр
.def __zero_reg__ = r2

.def ACCUMULATOR = r15


;-------------------------------------------
;                 Таймер T0                 |
;-------------------------------------------|
; time until Timer0 interrupt
#define Period_T0 (1)
; для режима CTC таймера
; Предделитель 64
#define CTC_OCR0 (Period_T0*F_CPU/(64*1000))



;====================================DATA======================================
.dseg
; Phisical inpunt/outputs (Xi/Yi)
IN_PORT: 			.byte		1
OUT_PORT:			.byte		1
; Bit variables (Mi)
MARKERS:			.byte		2
; Timer enable bits
TIMER1_EN:			.byte		1	; 1 ms
TIMER10_EN:			.byte		1	; 10 ms
TIMER100_EN:		.byte		1	; 100 ms
; Timer trigger bits
TIMER1_T:			.byte		1
TIMER10_T:			.byte		1
TIMER100_T:			.byte		1
; Storing the counting value of timers
TIMER1_POOL:		.byte		8*2
TIMER10_POOL:		.byte		8*2
TIMER100_POOL:		.byte		8*2
; PWM array
PWM_ARRAY:			.byte		3
; Data variables (Di)
DATA_VAR:			.byte		16*2
; Byte code
PLC_PROGRAM:		.byte		255


;====================================CODE======================================
.cseg
.org 0000
rjmp	RESET
.include "vectors_m16.inc"

;==============================================================================
;                           Обработчики прерываний
;                             Interrupt Handlers
;==============================================================================

;------------------------------------------------------------------------------
; Timer/Counter0 Compare Match Handler
; 1 ms interrupt
;------------------------------------------------------------------------------
TIM0_OC0_HANDLER:
			push	r16
			in		r16,SREG
			push	r16
			;----------




TIM0_OVF_HANDLER_EXIT:
			;----------
			pop		YH
			pop		r16
			out		SREG,r16
			pop		r16
			reti



;==============================================================================
; Main code
;==============================================================================
RESET:
			; Stack init
			ldi		r16, low(RAMEND)
			out		SPL, r16
			ldi		r16, high(RAMEND)
			out		SPH, r16

			; Обнуление памяти и регистров (объем кода: 80 байт прошивки)
			.include "coreinit.inc"

			; Нулевой регистр
			clr		__zero_reg__

			; Аналоговый компаратор выключен
			ldi		r16,1<<ACD
			out		ACSR,r16

			; Port A Init
			ldi		r16,0b00000000
			out		DDRA,r16
			ldi		r16,0b11111111
			out		PORTA,r16

			; Port B Init
			ldi		r16,0b11111111
			out		DDRB,r16
			ldi		r16,0b00000000
			out		PORTB,r16

			; Port C Init
			ldi		r16,0b00000000
			out		DDRC,r16
			ldi		r16,0b00000000
			out		PORTC,r16

			; Port D Init
			; PD4, PD5, PD7 - for PWM
			ldi 	r16,0b10110000
			out 	DDRD,r16
			ldi 	r16,0b00000000
			out 	PORTD,r16

			;------------------------------------------------------------------
			; CTC Mode for T0
			; Прерывание по совпадению каждую 1 мс
			;------------------------------------------------------------------
			ldi		r16,0
			OutReg	TCNT0,r16
			; Настройка предделителя 64, CTC Mode: WGM01 = 1, WGM00 = 0

			ldi		r16,(0<<CS02)|(1<<CS01)|(1<<CS00)|(1 << WGM01)
			OutReg	TCCR0,r16
			; OCR0 = CTC_OCRA
			ldi		r16,CTC_OCR0
			OutReg	OCR0,r16
			; Interrupt Timer/Counter0 Output Compare Match A
			ldi		r16,(1 << OCIE0)
			OutReg	TIMSK,r16
			;------------------------------------------------------------------

			rcall	PWM_INIT

			rcall	PLC_INIT

			; Загрузить программу из EEPROM
			rcall	EEPROM_LOAD_PROGRAM
			tst		r16
			brne	START_PLC_PROGRAM
			rjmp	PROGRAM_NOT_FOUND
START_PLC_PROGRAM:
			sei
			; Инициализиуем указатель на программу в RAM
			ldi		XL,low(PLC_PROGRAM)
			ldi		XH,high(PLC_PROGRAM)
			rjmp	PLC_CYCLE



PROGRAM_NOT_FOUND:
			rjmp	PROGRAM_NOT_FOUND





;------------------------------------------------------------------------------
; Load PLC program from EEPROM
;
; USED: r16*, r17*, r18*, r19*, r20*, XL*, XH*
; CALLS: EERead
; IN: -
; OUT: r16 = 1 - success
;------------------------------------------------------------------------------
EEPROM_LOAD_PROGRAM:
			ldi 	r16,low(EEPROM_TEST)	; Загружаем адрес ячейки EEPROM
			ldi 	r17,high(EEPROM_TEST)	; из которой хотим прочитать байт
			rcall 	EERead 					; (OUT: r18)
			cpi		r18,0xFF
			breq	EEPROM_EMPTY			; если равно 0xFF - память пуста
			; считать размер программы
			ldi 	r16,low(PROGRAM_SIZE)	; Загружаем адрес ячейки EEPROM
			ldi 	r17,high(PROGRAM_SIZE)	; из которой хотим прочитать байт
			rcall 	EERead 					; (OUT: r18)
			
			mov		r19,r18	; счетчик считанных байтов
			ldi 	r16,low(EEPROM_PLC_PROGRAM)		; Загружаем адрес ячейки EEPROM
			ldi 	r17,high(EEPROM_PLC_PROGRAM)	; из которой хотим прочитать байт
			ldi		XL,low(PLC_PROGRAM)
			ldi		XH,high(PLC_PROGRAM)
			ldi		r20,1
EEPROM_LOAD_PROGRAM_LOOP:
			rcall 	EERead 					; (OUT: r18)
			st		X+,r18
			add		r16,r20
			adc		r17,__zero_reg__
			dec		r19
			brne	EEPROM_LOAD_PROGRAM_LOOP
			ldi		r16,1
			ret
EEPROM_EMPTY:
			ldi		r16,0 ; r16 = 0 код ошибки
			ret



;------------------------------------------------------------------------------
; Main Loop
;------------------------------------------------------------------------------
PLC_CYCLE:
			rcall	READ_INPUTS
			rcall	vm_loop
			rcall	WRITE_OUTPUTS
			rjmp	PLC_CYCLE


;------------------------------------------------------------------------------
; PLC initialization
; 
;------------------------------------------------------------------------------
PLC_INIT:
			nop
			; инициализация входов
			; инициализация выходов
			; обнуление переменных
			ret


;------------------------------------------------------------------------------
; Read inputs
; 
;------------------------------------------------------------------------------
READ_INPUTS:
			in		r16,PINA
			com		r16
			sts		IN_PORT,r16
			ret


;------------------------------------------------------------------------------
; Write outputs
; 
;------------------------------------------------------------------------------
WRITE_OUTPUTS:
			lds		r16,OUT_PORT
			out		PORTB,r16
			ret


;------------------------------------------------------------------------------
; Start virtual machine
;
; USED: r0*, r1*, r16*, r24*, r25*, ZL*, ZH*
; CALLS: -
; IN: r16 - size of program
; OUT: -
;------------------------------------------------------------------------------
vm_loop:
			ld		r0,X+
			ldi		r16,2
			mul		r0,r16
			ldi		ZL,low(2*VM_OPERATIONS)
			ldi		ZH,high(2*VM_OPERATIONS)
			add		ZL,r0
			adc		ZH,r1
			lpm		r24,Z+
			lpm		r25,Z
			movw	ZL,r24	; теперь Z указывает на адрес подпрограммы
			ijmp			; косвенный переход к подпрограмме
			; в конце ret не ставится, возврат делается из команды


;------------------------------------------------------------------------------
; Virtual machine NOP instruction
; SYNTAX: NOP
;
; ARGS: 0
; USED: -
; CALLS: -
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_nop:
			nop
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine LD instruction
; SYNTAX: LD <input_pin>
;
; ARGS: 1 (Xi, Yi, Mi)
; USED: r16*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_ld_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_ld
vm_ld_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_ld
vm_ld_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_ld:
			ld		r16,X+
			rcall	BitRead
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine LDN instruction
; SYNTAX: LDN <input_pin>
;
; ARGS: 1 (Xi, Yi, Mi)
; USED: r16*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_ldn_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_ldn
vm_ldn_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_ldn
vm_ldn_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_ldn:
			ld		r16,X+
			rcall	BitRead
			; inverse bit in ACCUMULATOR
			ldi		r16,1
			eor		ACCUMULATOR,r16
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine ST instruction
; SYNTAX: ST <output_pin>
;
; ARGS: 1 (Yi, Mi)
; USED: r16*, YL*, YH*
; CALLS: BitWrite
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_st_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_st
vm_st_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
vm_st:
			ld		r16,X+
			rcall	BitWrite
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine STN instruction
; SYNTAX: STN <output_pin>
;
; ARGS: 1 (Yi, Mi)
; USED: r16*, YL*, YH*
; CALLS: BitWrite
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_stn_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_stn
vm_stn_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
vm_stn:
			ld		r16,X+
			rcall	BitWrite
			; inverse bit in ACCUMULATOR
			ldi		r16,1
			eor		ACCUMULATOR,r16
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine AND instruction
; Boolean AND contents of accumulator and Xi, Yi, Mi
; SYNTAX: AND [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_and_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_and
vm_and_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_and
vm_and_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_and:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			and		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine ANDN instruction
; Boolean AND contents of accumulator and inverse of Xi, Yi, Mi
; SYNTAX: ANDN [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_andn_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_andn
vm_andn_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_andn
vm_andn_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_andn:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			; inverse bit
			ldi		r16,1
			eor		ACCUMULATOR,r16
			; AND with previous content of ACCUMULATOR
			and		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine OR instruction
; Boolean OR contents of accumulator and Xi, Yi, Mi
; SYNTAX: OR [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_or_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_or
vm_or_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_or
vm_or_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_or:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			or		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine ORN instruction
; Boolean OR contents of accumulator and inverse of Xi, Yi, Mi
; SYNTAX: ORN [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_orn_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_orn
vm_orn_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_orn
vm_orn_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_orn:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			; inverse bit
			ldi		r16,1
			eor		ACCUMULATOR,r16
			; OR with previous content of ACCUMULATOR
			or		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine XOR instruction
; 
; SYNTAX: XOR [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_xor_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_xor
vm_xor_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_xor
vm_xor_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_xor:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			eor		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine XORN instruction
; 
; SYNTAX: XORN [Xi, Yi, Mi]
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: BitRead
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_xorn_m:
			ldi		YL,low(MARKERS)
			ldi		YH,high(MARKERS)
			rjmp	vm_xorn
vm_xorn_y:
			ldi		YL,low(OUT_PORT)
			ldi		YH,high(OUT_PORT)
			rjmp	vm_xorn
vm_xorn_x:
			ldi		YL,low(IN_PORT)
			ldi		YH,high(IN_PORT)
vm_xorn:
			mov		r18,ACCUMULATOR
			ld		r16,X+
			rcall	BitRead
			; inverse bit
			ldi		r16,1
			eor		ACCUMULATOR,r16
			; OR with previous content of ACCUMULATOR
			eor		ACCUMULATOR,r18
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine NOT instruction
; Boolean NOT of ACCUMULATOR
; SYNTAX: NOT
;
; ARGS: 0
; USED: r16*
; CALLS: -
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_not:
			ldi		r16,1
			eor		ACCUMULATOR,r16
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine END instruction
; End program, return to input/output process and 0 step.
; SYNTAX: END
;
; ARGS: 0
; USED: 
; CALLS: -
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_end:
			ldi		XL,low(PLC_PROGRAM)
			ldi		XH,high(PLC_PROGRAM)
			ret


;------------------------------------------------------------------------------
; Bit Read in IN_PORT
; считать состояние отдельного бита в байте
;
; USED: r16*, r17*, YL*, YH*, ACCUMULATOR
; CALLS: -
; IN: r16 номер бита bit
; OUT: ACCUMULATOR
;------------------------------------------------------------------------------
; Y - указатель на переменную
; r16 - номер бита
; r17 - байт
BitRead:
			clr		ACCUMULATOR
			cpi		r16,8
			brsh	BitRead_next_byte
			rjmp	BitRead_do
BitRead_next_byte:
			subi	r16,8
			adiw	Y,1
BitRead_do:
			ld		r17,Y
BitRead_loop:
			lsr		r17  ; бит во флаге переноса
			dec		r16
			brge	BitRead_loop
			brcc	BitRead_exit
			inc		ACCUMULATOR
BitRead_exit:
			ret


;------------------------------------------------------------------------------
; Bit Write in OUT_PORT
; записать отдельный бит в байте
;
; USED: r16*, r17*, r18*, YL*, YH*, ACCUMULATOR
; CALLS: -
; IN: r16 номер бита bit
; OUT: 
;------------------------------------------------------------------------------
BitWrite:
			; Y - указатель на переменную
			; r15 - 0/1 - записываемый бит
			; r16 - номер бита
			; r17 - байт для модификации
			; r18 - маска
			cpi		r16,8
			brsh	BitWrite_next_byte
			rjmp	BitWrite_do
BitWrite_next_byte:
			subi	r16,8
			adiw	Y,1
BitWrite_do:
			ld		r17,Y
			clr		r18
			sec		; Установить флаг переноса
BitWrite_loop:
			rol		r18 ; вставляем справа бит из флага переноса
			dec		r16
			brge	BitWrite_loop
			tst		ACCUMULATOR
			breq	BitWrite_rst
			or		r17,r18
			rjmp	BitWrite_save
BitWrite_rst:
			com		r18
			and		r17,r18
BitWrite_save:
			st		Y,r17
			ret


;------------------------------------------------------------------------------
; Virtual machine ADD instruction
; 
; SYNTAX: ADD Di Ki
; SYNTAX: ADD Di Di
;
; ARGS: 2
; USED: r16*, r17*, r18*, r19*, XL*, XH*, YL*, YH*
; CALLS: 
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_add_k:
			tst		ACCUMULATOR
			brne	vm_add_k_do
			ld		r16,X+
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_add_k_do:
			ld		r16,X+	; first term (var id)
			ld		r18,X+	; const Hi-byte
			ld		r19,X+	; const Lo-byte
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			; Load value of var
			ld		r16,Y+	; var Hi-byte
			ld		r17,Y+	; var Lo-byte
			; addition
			add		r17,r19
			adc		r16,r18
			; Store value to var
			st		-Y,r17
			st		-Y,r16
			rjmp	vm_loop
;------------------------------------------------------------------------------
; SYNTAX: ADD Di Di
;------------------------------------------------------------------------------
vm_add_d:
			tst		ACCUMULATOR
			brne	vm_add_d_do
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_add_d_do:
			ld		r16,X+	; first term (var id)
			ld		r17,X+	; second term (var id)
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r17
			adc		YH,__zero_reg__
			; Load second value
			ld		r18,Y+	; second term Hi-byte
			ld		r19,Y+	; second term Lo-byte
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			ld		r16,Y+	; first term Hi-byte
			ld		r17,Y+	; first term Lo-byte
			; addition
			add		r17,r19
			adc		r16,r18
			; Store value to var
			st		-Y,r17
			st		-Y,r16
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine SUB instruction
; 
; SYNTAX: SUB Di Ki
; SYNTAX: SUB Di Di
;
; ARGS: 2
; USED: r16*, r17*, r18*, r19*, XL*, XH*, YL*, YH*
; CALLS: 
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_sub_k:
			tst		ACCUMULATOR
			brne	vm_sub_k_do
			ld		r16,X+
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_sub_k_do:
			ld		r16,X+	; minuend (var id)
			ld		r18,X+	; subtrahend const Hi-byte
			ld		r19,X+	; subtrahend const Lo-byte
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			; Load value of var
			ld		r16,Y+	; var Hi-byte
			ld		r17,Y+	; var Lo-byte
			; subtraction
			sub		r17,r19
			sbc		r16,r18
			; Store value to var
			st		-Y,r17
			st		-Y,r16
			rjmp	vm_loop
;------------------------------------------------------------------------------
; SYNTAX: SUB Di Di
;------------------------------------------------------------------------------
vm_sub_d:
			tst		ACCUMULATOR
			brne	vm_sub_d_do
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_sub_d_do:
			ld		r16,X+	; minuend (var id)
			ld		r17,X+	; subtrahend (var id)
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r17
			adc		YH,__zero_reg__
			; Load second value
			ld		r18,Y+	; subtrahend Hi-byte
			ld		r19,Y+	; subtrahend Lo-byte
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			ld		r16,Y+	; minuend Hi-byte
			ld		r17,Y+	; minuend Lo-byte
			; subtraction
			sub		r17,r19
			sbc		r16,r18
			; Store value to var
			st		-Y,r17
			st		-Y,r16
			rjmp	vm_loop


; Заглушка для пока не реализованных подпрограмм
vm_mul_k:
vm_mul_d:
vm_div_k:
vm_div_d:
vm_mod_k:
vm_mod_d:
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine PWM instruction
; 
; SYNTAX: PWM Ki[8] PYi
; SYNTAX: PWM Di PYi
;
; ARGS: 1
; USED: r16*, r18*, YL*, YH*
; CALLS: 
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_pwm_k:
			tst		ACCUMULATOR
			brne	vm_pwm_k_do
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_pwm_k_do:
			ld		r16,X+	; duty cycle (const)
			ld		r17,X+	; output
			ldi		YL,low(PWM_ARRAY)
			ldi		YH,high(PWM_ARRAY)
			add		YL,r17
			adc		YH,__zero_reg__
			st		Y,r16
			rjmp	vm_loop
;------------------------------------------------------------------------------
; SYNTAX: PWM Di PYi
;------------------------------------------------------------------------------
vm_pwm_d:
			tst		ACCUMULATOR
			brne	vm_pwm_d_do
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_pwm_d_do:
			ld		r16,X+	; duty cycle (var id)
			ld		r17,X+	; output
			; Load duty cycle value from var
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			ld		r16,Y+	; extract Hi-byte (it doesn't matter)
			ld		r16,Y	; extract Lo-byte (PWM duty cycle)
			; Save value to PWM_ARRAY
			ldi		YL,low(PWM_ARRAY)
			ldi		YH,high(PWM_ARRAY)
			add		YL,r17
			adc		YH,__zero_reg__
			st		Y,r16
			rjmp	vm_loop


;------------------------------------------------------------------------------
; Virtual machine MOV instruction
; 
; SYNTAX: MOV Di Ki[16]
; SYNTAX: MOV Di Di
;
; ARGS: 1
; USED: r16*, r17*, r18*, XL*, XH*, YL*, YH*
; CALLS: 
; IN: -
; OUT: -
;------------------------------------------------------------------------------
vm_mov_k:
			tst		ACCUMULATOR
			brne	vm_mov_k_do
			ld		r16,X+
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_mov_k_do:
			ld		r16,X+	; destination (var id)
			ld		r17,X+	; source (const) Hi-byte
			ld		r18,X+	; source (const) Lo-byte
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			st		Y+,r17	; store Hi-byte
			st		Y,r18	; store Lo-byte
			rjmp	vm_loop
;------------------------------------------------------------------------------
; SYNTAX: MOV Di Di
;------------------------------------------------------------------------------
vm_mov_d:
			tst		ACCUMULATOR
			brne	vm_mov_d_do
			ld		r16,X+
			ld		r16,X+
			rjmp	vm_loop
vm_mov_d_do:
			ld		r16,X+	; destination (var id)
			ld		r17,X+	; source (var id)
			; Load source value
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r17
			adc		YH,__zero_reg__
			ld		r17,Y+	; extract Hi-byte of source value
			ld		r18,Y	; extract Lo-byte of source value
			; Store value to destination var
			ldi		YL,low(DATA_VAR)
			ldi		YH,high(DATA_VAR)
			add		YL,r16
			adc		YH,__zero_reg__
			st		Y+,r17	; store Hi-byte
			st		Y,r18	; store Lo-byte
			rjmp	vm_loop


.include "eeprom.asm"
.include "pwm.asm"


;============================= FLASH Constants ==============================
; Таблица адресов команд
VM_OPERATIONS:
.db low(vm_nop), high(vm_nop)      ; 0x00  NOP
.db low(vm_ld_x), high(vm_ld_x)    ; 0x01  LD Xi
.db low(vm_ld_y), high(vm_ld_y)    ; 0x02  LD Yi
.db low(vm_ld_m), high(vm_ld_m)    ; 0x03  LD Mi
.db low(vm_ldn_x), high(vm_ldn_x)  ; 0x04  LDN Xi
.db low(vm_ldn_y), high(vm_ldn_y)  ; 0x05  LDN Yi
.db low(vm_ldn_m), high(vm_ldn_m)  ; 0x06  LDN Mi
.db low(vm_st_y), high(vm_st_y)    ; 0x07  ST Yi
.db low(vm_st_m), high(vm_st_m)    ; 0x08  ST Mi
.db low(vm_stn_y), high(vm_stn_y)  ; 0x09  STN Yi
.db low(vm_stn_m), high(vm_stn_m)  ; 0x0A  STN Mi
.db low(vm_and_x), high(vm_and_x)  ; 0x0B  AND Xi
.db low(vm_and_y), high(vm_and_y)  ; 0x0C  AND Yi
.db low(vm_and_m), high(vm_and_m)  ; 0x0D  AND Mi
.db low(vm_andn_x), high(vm_andn_x); 0x0E  ANDN Xi
.db low(vm_andn_y), high(vm_andn_y); 0x0F  ANDN Yi
.db low(vm_andn_m), high(vm_andn_m); 0x10  ANDN Mi
.db low(vm_or_x), high(vm_or_x)    ; 0x11  OR Xi
.db low(vm_or_y), high(vm_or_y)    ; 0x12  OR Yi
.db low(vm_or_m), high(vm_or_m)    ; 0x13  OR Mi
.db low(vm_orn_x), high(vm_orn_x)  ; 0x14  ORN Xi
.db low(vm_orn_y), high(vm_orn_y)  ; 0x15  ORN Yi
.db low(vm_orn_m), high(vm_orn_m)  ; 0x16  ORN Mi
.db low(vm_xor_x), high(vm_xor_x)  ; 0x17  XOR Xi
.db low(vm_xor_y), high(vm_xor_y)  ; 0x18  XOR Yi
.db low(vm_xor_m), high(vm_xor_m)  ; 0x19  XOR Mi
.db low(vm_xorn_x), high(vm_xorn_x); 0x1A  XORN Xi
.db low(vm_xorn_y), high(vm_xorn_y); 0x1B  XORN Yi
.db low(vm_xorn_m), high(vm_xorn_m); 0x1C  XORN Mi
.db low(vm_not), high(vm_not)      ; 0x1D  NOT
.db low(vm_add_k), high(vm_add_k)  ; 0x1E  ADD Di Ki
.db low(vm_add_d), high(vm_add_d)  ; 0x1F  ADD Di Di
.db low(vm_sub_k), high(vm_sub_k)  ; 0x20  SUB Di Ki
.db low(vm_sub_d), high(vm_sub_d)  ; 0x21  SUB Di Di
.db low(vm_mul_k), high(vm_mul_k)  ; 0x22  MUL Di Ki
.db low(vm_mul_d), high(vm_mul_d)  ; 0x23  MUL Di Di
.db low(vm_div_k), high(vm_div_k)  ; 0x24  DIV Di Ki
.db low(vm_div_d), high(vm_div_d)  ; 0x25  DIV Di Di
.db low(vm_mod_k), high(vm_mod_k)  ; 0x26  MOD Di Ki
.db low(vm_mod_d), high(vm_mod_d)  ; 0x27  MOD Di Di
.db low(vm_pwm_k), high(vm_pwm_k)  ; 0x28  PWM Ki[8] PYi
.db low(vm_pwm_d), high(vm_pwm_d)  ; 0x29  PWM Di PYi
.db low(vm_mov_k), high(vm_mov_k)  ; 0x2A  MOV Di Ki[16]
.db low(vm_mov_d), high(vm_mov_d)  ; 0x2B  MOV Di Di
.db low(vm_end), high(vm_end)      ; 0x2C  END



;================================= EEPROM ===================================
.eseg
.org 0x100
EEPROM_TEST:		.db 0   ; для проверки, если равно 0xFF, то EEPROM чиста
PROGRAM_SIZE:		.db 21  ; размер программы в байтах
EEPROM_PLC_PROGRAM:
; Single Push button
.db 0x01, 0x00   ; LD X0
.db 0x0F, 0x00   ; ANDN Y0
.db 0x08, 0x01   ; ST M1
; 
.db 0x01, 0x00   ; LD X0
.db 0x0C, 0x00   ; AND Y0
.db 0x08, 0x02   ; ST M2
; 
.db 0x03, 0x01   ; LD M1
.db 0x12, 0x00   ; OR Y0
.db 0x10, 0x02   ; ANDN M2
.db 0x07, 0x00   ; ST Y0
.db 0x2C         ; END

