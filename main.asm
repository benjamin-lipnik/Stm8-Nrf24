;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.0.0 #11528 (Linux)
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _clear_status
	.globl _get_status
	.globl _flush_rx
	.globl _flush_tx
	.globl _read_payload_data
	.globl _read_register_block
	.globl _write_register_block
	.globl _delay_milliseconds
	.globl _uart_read
	.globl _uart_write
	.globl _uart_init
	.globl _SPI_read
	.globl _SPI_write
	.globl _SPI_init
	.globl _SPI_chip_deselect
	.globl _SPI_chip_select
	.globl _printf
	.globl _payload_buffer
	.globl _putchar
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_payload_buffer::
	.ds 5
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)

; default segment ordering for linker
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area CONST
	.area INITIALIZER
	.area CODE

;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ; reset
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	uart.h: 6: void uart_init(unsigned int baudrate) {
;	-----------------------------------------
;	 function uart_init
;	-----------------------------------------
_uart_init:
	sub	sp, #8
;	uart.h: 8: uint16_t div = (F_CPU + baudrate / 2) / baudrate;
	ldw	x, (0x0b, sp)
	srlw	x
	clr	a
	clr	(0x01, sp)
	addw	x, #0x2400
	ldw	(0x07, sp), x
	adc	a, #0xf4
	ld	(0x06, sp), a
	ld	a, (0x01, sp)
	adc	a, #0x00
	ld	(0x05, sp), a
	ldw	y, (0x0b, sp)
	clrw	x
	pushw	y
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	ldw	x, (0x0b, sp)
	pushw	x
	call	__divulong
	addw	sp, #8
;	uart.h: 10: UART1_BRR2 = ((div >> 8) & 0xF0) + (div & 0x0F);
	ld	a, xh
	clr	(0x07, sp)
	and	a, #0xf0
	ld	(0x08, sp), a
	ld	a, xl
	and	a, #0x0f
	add	a, (0x08, sp)
	ld	0x5233, a
;	uart.h: 11: UART1_BRR1 = div >> 4;
	ld	a, #0x10
	div	x, a
	ld	a, xl
	ld	0x5232, a
;	uart.h: 13: UART1_CR2 = (1 << UART1_CR2_TEN) | (1 << UART1_CR2_REN);
	mov	0x5235+0, #0x0c
;	uart.h: 14: }
	addw	sp, #8
	ret
;	uart.h: 16: void uart_write(uint8_t data) {
;	-----------------------------------------
;	 function uart_write
;	-----------------------------------------
_uart_write:
;	uart.h: 17: UART1_DR = data;
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	uart.h: 18: while (!(UART1_SR & (1 << UART1_SR_TC)));
00101$:
	ld	a, 0x5230
	bcp	a, #0x40
	jreq	00101$
;	uart.h: 19: }
	ret
;	uart.h: 21: uint8_t uart_read() {
;	-----------------------------------------
;	 function uart_read
;	-----------------------------------------
_uart_read:
;	uart.h: 22: while (!(UART1_SR & (1 << UART1_SR_RXNE)));
00101$:
	ld	a, 0x5230
	bcp	a, #0x20
	jreq	00101$
;	uart.h: 23: return UART1_DR;
	ld	a, 0x5231
;	uart.h: 24: }
	ret
;	main.c: 9: void delay_milliseconds(unsigned long milliseconds) {
;	-----------------------------------------
;	 function delay_milliseconds
;	-----------------------------------------
_delay_milliseconds:
	push	a
;	main.c: 11: for(uint8_t m = 0; m < milliseconds; m++) {
	clr	(0x01, sp)
00111$:
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	clrw	y
	cpw	x, (0x06, sp)
	ld	a, yl
	sbc	a, (0x05, sp)
	ld	a, yh
	sbc	a, (0x04, sp)
	jrnc	00113$
;	main.c: 12: for(uint8_t i = 0; i < 160; i++) {
	clrw	x
00108$:
	ld	a, xl
	cp	a, #0xa0
	jrnc	00112$
;	main.c: 13: for(uint8_t j = 0; j < 10; j++) {
	clr	a
00105$:
	cp	a, #0x0a
	jrnc	00109$
;	main.c: 14: __asm__("nop");
	nop
;	main.c: 15: __asm__("nop");
	nop
;	main.c: 16: __asm__("nop");
	nop
;	main.c: 17: __asm__("nop");
	nop
;	main.c: 13: for(uint8_t j = 0; j < 10; j++) {
	inc	a
	jra	00105$
00109$:
;	main.c: 12: for(uint8_t i = 0; i < 160; i++) {
	incw	x
	jra	00108$
00112$:
;	main.c: 11: for(uint8_t m = 0; m < milliseconds; m++) {
	inc	(0x01, sp)
	jra	00111$
00113$:
;	main.c: 21: }
	pop	a
	ret
;	main.c: 30: void write_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function write_register_block
;	-----------------------------------------
_write_register_block:
	push	a
;	main.c: 31: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 32: SPI_write(W_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x04, sp)
	and	a, #0x1f
	or	a, #0x20
	push	a
	call	_SPI_write
	pop	a
;	main.c: 33: while(size--) {
	ldw	x, (0x05, sp)
	ld	a, (0x07, sp)
	ld	(0x01, sp), a
00101$:
	ld	a, (0x01, sp)
	dec	(0x01, sp)
	tnz	a
	jreq	00103$
;	main.c: 34: SPI_write(*(buffer++));
	ld	a, (x)
	incw	x
	pushw	x
	push	a
	call	_SPI_write
	pop	a
	popw	x
	jra	00101$
00103$:
;	main.c: 36: SPI_chip_deselect();
	pop	a
	jp	_SPI_chip_deselect
;	main.c: 37: }
	pop	a
	ret
;	main.c: 38: void read_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function read_register_block
;	-----------------------------------------
_read_register_block:
	push	a
;	main.c: 39: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 40: SPI_write(R_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x04, sp)
	and	a, #0x1f
	push	a
	call	_SPI_write
	pop	a
;	main.c: 41: while(size--) {
	ldw	x, (0x05, sp)
	ld	a, (0x07, sp)
	ld	(0x01, sp), a
00101$:
	ld	a, (0x01, sp)
	dec	(0x01, sp)
	tnz	a
	jreq	00103$
;	main.c: 42: *(buffer++) = SPI_read();
	pushw	x
	call	_SPI_read
	popw	x
	ld	(x), a
	incw	x
	jra	00101$
00103$:
;	main.c: 44: SPI_chip_deselect();
	pop	a
	jp	_SPI_chip_deselect
;	main.c: 45: }
	pop	a
	ret
;	main.c: 46: void read_payload_data (uint8_t command, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function read_payload_data
;	-----------------------------------------
_read_payload_data:
	push	a
;	main.c: 47: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 48: SPI_write(command);
	ld	a, (0x04, sp)
	push	a
	call	_SPI_write
	pop	a
;	main.c: 49: while(size--) {
	ldw	x, (0x05, sp)
	ld	a, (0x07, sp)
	ld	(0x01, sp), a
00101$:
	ld	a, (0x01, sp)
	dec	(0x01, sp)
	tnz	a
	jreq	00103$
;	main.c: 50: *(buffer++) = SPI_read();
	pushw	x
	call	_SPI_read
	popw	x
	ld	(x), a
	incw	x
	jra	00101$
00103$:
;	main.c: 52: SPI_chip_deselect();
	pop	a
	jp	_SPI_chip_deselect
;	main.c: 53: }
	pop	a
	ret
;	main.c: 54: void flush_tx() {
;	-----------------------------------------
;	 function flush_tx
;	-----------------------------------------
_flush_tx:
;	main.c: 55: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 56: SPI_write(FLUSH_TX);
	push	#0xe1
	call	_SPI_write
	pop	a
;	main.c: 57: SPI_chip_deselect();
;	main.c: 58: }
	jp	_SPI_chip_deselect
;	main.c: 59: void flush_rx() {
;	-----------------------------------------
;	 function flush_rx
;	-----------------------------------------
_flush_rx:
;	main.c: 60: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 61: SPI_write(FLUSH_RX);
	push	#0xe2
	call	_SPI_write
	pop	a
;	main.c: 62: SPI_chip_deselect();
;	main.c: 63: }
	jp	_SPI_chip_deselect
;	main.c: 64: uint8_t get_status () {
;	-----------------------------------------
;	 function get_status
;	-----------------------------------------
_get_status:
;	main.c: 65: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 66: SPI_write(R_REGISTER | (REGISTER_MASK & STATUS_NRF));
	push	#0x07
	call	_SPI_write
	pop	a
;	main.c: 67: uint8_t ret = SPI_read();
	call	_SPI_read
;	main.c: 68: SPI_chip_deselect();
	push	a
	call	_SPI_chip_deselect
	pop	a
;	main.c: 69: return ret;
;	main.c: 70: }
	ret
;	main.c: 71: void clear_status () {
;	-----------------------------------------
;	 function clear_status
;	-----------------------------------------
_clear_status:
;	main.c: 72: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 73: SPI_write(W_REGISTER | (REGISTER_MASK & STATUS_NRF));
	push	#0x27
	call	_SPI_write
	pop	a
;	main.c: 74: SPI_write(_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	push	#0x70
	call	_SPI_write
	pop	a
;	main.c: 75: SPI_chip_deselect();
;	main.c: 76: }
	jp	_SPI_chip_deselect
;	main.c: 78: int putchar(int c) {
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
_putchar:
;	main.c: 79: uart_write(c);
	ld	a, (0x04, sp)
	push	a
	call	_uart_write
	pop	a
;	main.c: 80: return 0;
	clrw	x
;	main.c: 81: }
	ret
;	main.c: 83: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #12
;	main.c: 84: CLK_CKDIVR = 0; //16Mhz
	mov	0x50c6+0, #0x00
;	main.c: 86: uart_init(9600);
	push	#0x80
	push	#0x25
	call	_uart_init
	addw	sp, #2
;	main.c: 88: PC_DDR |= _BV(CSN_PIN);
	bset	20492, #4
;	main.c: 89: PC_CR1 |= _BV(CSN_PIN);
	bset	20493, #4
;	main.c: 90: PC_ODR |= _BV(CSN_PIN);
	bset	20490, #4
;	main.c: 95: SPI_init();
	call	_SPI_init
;	main.c: 97: while(1) {
00104$:
;	main.c: 98: printf("Initting radio.\n\r");
	push	#<(___str_0 + 0)
	push	#((___str_0 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 100: delay_milliseconds(100); //OFF_TO_POWERDOWN_MILLIS
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 103: uint8_t channel = 100;
	ld	a, #0x64
	ld	(0x01, sp), a
;	main.c: 104: write_register_block(RF_CH, &channel, 1);
	ldw	x, sp
	incw	x
	push	#0x01
	pushw	x
	push	#0x05
	call	_write_register_block
	addw	sp, #4
;	main.c: 106: uint8_t rf_setup = RF_TX_PWR_MAX | _BV(3);
	ld	a, #0x0e
	ld	(0x02, sp), a
;	main.c: 107: write_register_block(RF_SETUP, &rf_setup, 1);
	ldw	x, sp
	incw	x
	incw	x
	push	#0x01
	pushw	x
	push	#0x06
	call	_write_register_block
	addw	sp, #4
;	main.c: 109: uint8_t address[] = {7,7,7,7,7};
	ld	a, #0x07
	ld	(0x03, sp), a
	ldw	x, sp
	ld	a, #0x07
	ld	(4, x), a
	ldw	x, sp
	ld	a, #0x07
	ld	(5, x), a
	ldw	x, sp
	ld	a, #0x07
	ld	(6, x), a
	ldw	x, sp
	ld	a, #0x07
	ld	(7, x), a
;	main.c: 110: write_register_block(RX_ADDR_P1, address, 5);
	push	#0x05
	ldw	x, sp
	addw	x, #4
	pushw	x
	push	#0x0b
	call	_write_register_block
	addw	sp, #4
;	main.c: 112: uint8_t dynpd = _BV(DPL_P0) | _BV(DPL_P1);
	ld	a, #0x03
	ld	(0x08, sp), a
;	main.c: 113: write_register_block(DYNPD, &dynpd, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #9
	pushw	x
	push	#0x1c
	call	_write_register_block
	addw	sp, #4
;	main.c: 115: uint8_t feature = _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK);
	ld	a, #0x07
	ld	(0x09, sp), a
;	main.c: 116: write_register_block(FEATURE, &feature, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #10
	pushw	x
	push	#0x1d
	call	_write_register_block
	addw	sp, #4
;	main.c: 118: flush_rx();
	call	_flush_rx
;	main.c: 119: flush_tx();
	call	_flush_tx
;	main.c: 121: clear_status();
	call	_clear_status
;	main.c: 124: uint8_t config = CONFIG_REG_SETTINGS_FOR_RX_MODE;
	ld	a, #0x0b
	ld	(0x0a, sp), a
;	main.c: 125: write_register_block(CONFIG, &config, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #11
	pushw	x
	push	#0x00
	call	_write_register_block
	addw	sp, #4
;	main.c: 127: delay_milliseconds(5);
	push	#0x05
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 129: uint8_t config_check = 0;
	clr	(0x0b, sp)
;	main.c: 130: read_register_block(CONFIG, &config_check, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #12
	pushw	x
	push	#0x00
	call	_read_register_block
	addw	sp, #4
;	main.c: 132: if(config_check == CONFIG_REG_SETTINGS_FOR_RX_MODE) {
	ld	a, (0x0b, sp)
	cp	a, #0x0b
	jreq	00157$
	jp	00104$
00157$:
;	main.c: 136: delay_milliseconds(10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 137: printf("Init successufull!!!\n\r");
	push	#<(___str_1 + 0)
	push	#((___str_1 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 139: while(1) {
00112$:
;	main.c: 140: uint8_t status = get_status();
	call	_get_status
;	main.c: 143: if((status & 0b1110) == 0b1110) {//no data
	and	a, #0x0e
	ld	xl, a
	clr	a
	ld	xh, a
	cpw	x, #0x000e
	jrne	00107$
;	main.c: 144: delay_milliseconds(40);
	push	#0x28
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 145: continue;
	jra	00112$
00107$:
;	main.c: 148: read_payload_data(R_RX_PAYLOAD, payload_buffer, PAYLOAD_SIZE);
	push	#0x05
	push	#<(_payload_buffer + 0)
	push	#((_payload_buffer + 0) >> 8)
	push	#0x61
	call	_read_payload_data
	addw	sp, #4
;	main.c: 150: if(get_status() & _BV(RX_DR)) {
	call	_get_status
	bcp	a, #0x40
	jreq	00109$
;	main.c: 151: clear_status();
	call	_clear_status
00109$:
;	main.c: 154: printf("data: ");
	push	#<(___str_2 + 0)
	push	#((___str_2 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 155: for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
	clr	(0x0c, sp)
00115$:
	ld	a, (0x0c, sp)
	cp	a, #0x05
	jrnc	00110$
;	main.c: 156: printf("%u, ", payload_buffer[i]);
	clrw	x
	ld	a, (0x0c, sp)
	ld	xl, a
	addw	x, #(_payload_buffer + 0)
	ld	a, (x)
	clrw	x
	ld	xl, a
	pushw	x
	push	#<(___str_3 + 0)
	push	#((___str_3 + 0) >> 8)
	call	_printf
	addw	sp, #4
;	main.c: 155: for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
	inc	(0x0c, sp)
	jra	00115$
00110$:
;	main.c: 158: printf("\n\r");
	push	#<(___str_4 + 0)
	push	#((___str_4 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 160: delay_milliseconds(1);
	push	#0x01
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
	jra	00112$
;	main.c: 204: }
	addw	sp, #12
	ret
	.area CODE
	.area CONST
	.area CONST
___str_0:
	.ascii "Initting radio."
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_1:
	.ascii "Init successufull!!!"
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_2:
	.ascii "data: "
	.db 0x00
	.area CODE
	.area CONST
___str_3:
	.ascii "%u, "
	.db 0x00
	.area CODE
	.area CONST
___str_4:
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
