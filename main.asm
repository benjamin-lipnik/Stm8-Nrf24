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
	.globl _printbin
	.globl _uart_read
	.globl _uart_write
	.globl _uart_init
	.globl _SPI_transfer
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
;	main.c: 9: void printbin(uint8_t dat) {
;	-----------------------------------------
;	 function printbin
;	-----------------------------------------
_printbin:
	sub	sp, #3
;	main.c: 10: for(uint8_t i = 0; i < 8; i++) {
	clr	(0x03, sp)
00103$:
	ld	a, (0x03, sp)
	cp	a, #0x08
	jrnc	00105$
;	main.c: 12: uart_write(48 + ((dat&_BV(i))>0));
	clrw	x
	incw	x
	ld	a, (0x03, sp)
	jreq	00120$
00119$:
	sllw	x
	dec	a
	jrne	00119$
00120$:
	ld	a, (0x06, sp)
	ld	(0x02, sp), a
	clr	(0x01, sp)
	ld	a, xl
	and	a, (0x02, sp)
	rlwa	x
	and	a, (0x01, sp)
	ld	xh, a
	cpw	x, #0x0000
	jrsgt	00121$
	clr	a
	.byte 0xc5
00121$:
	ld	a, #0x01
00122$:
	add	a, #0x30
	push	a
	call	_uart_write
	pop	a
;	main.c: 10: for(uint8_t i = 0; i < 8; i++) {
	inc	(0x03, sp)
	jra	00103$
00105$:
;	main.c: 14: }
	addw	sp, #3
	ret
;	main.c: 16: void delay_milliseconds(unsigned long milliseconds) { //for 16Mhz
;	-----------------------------------------
;	 function delay_milliseconds
;	-----------------------------------------
_delay_milliseconds:
	push	a
;	main.c: 18: for(uint8_t m = 0; m < milliseconds; m++) {
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
;	main.c: 19: for(uint8_t i = 0; i < 160; i++) {
	clrw	x
00108$:
	ld	a, xl
	cp	a, #0xa0
	jrnc	00112$
;	main.c: 20: for(uint8_t j = 0; j < 10; j++) {
	clr	a
00105$:
	cp	a, #0x0a
	jrnc	00109$
;	main.c: 21: __asm__("nop");
	nop
;	main.c: 22: __asm__("nop");
	nop
;	main.c: 23: __asm__("nop");
	nop
;	main.c: 24: __asm__("nop");
	nop
;	main.c: 20: for(uint8_t j = 0; j < 10; j++) {
	inc	a
	jra	00105$
00109$:
;	main.c: 19: for(uint8_t i = 0; i < 160; i++) {
	incw	x
	jra	00108$
00112$:
;	main.c: 18: for(uint8_t m = 0; m < milliseconds; m++) {
	inc	(0x01, sp)
	jra	00111$
00113$:
;	main.c: 28: }
	pop	a
	ret
;	main.c: 38: void write_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function write_register_block
;	-----------------------------------------
_write_register_block:
	push	a
;	main.c: 39: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 40: SPI_transfer(W_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x04, sp)
	and	a, #0x1f
	or	a, #0x20
	push	a
	call	_SPI_transfer
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
;	main.c: 42: SPI_transfer(*(buffer++));
	ld	a, (x)
	incw	x
	pushw	x
	push	a
	call	_SPI_transfer
	pop	a
	popw	x
	jra	00101$
00103$:
;	main.c: 44: SPI_chip_deselect();
	pop	a
	jp	_SPI_chip_deselect
;	main.c: 45: }
	pop	a
	ret
;	main.c: 46: void read_register_block(uint8_t reg, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function read_register_block
;	-----------------------------------------
_read_register_block:
	push	a
;	main.c: 47: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 48: SPI_transfer(R_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x04, sp)
	and	a, #0x1f
	push	a
	call	_SPI_transfer
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
;	main.c: 50: *(buffer++) = SPI_transfer(0xff);
	pushw	x
	push	#0xff
	call	_SPI_transfer
	addw	sp, #1
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
;	main.c: 54: void read_payload_data (uint8_t command, uint8_t * buffer, uint8_t size) {
;	-----------------------------------------
;	 function read_payload_data
;	-----------------------------------------
_read_payload_data:
	push	a
;	main.c: 55: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 56: SPI_transfer(command);
	ld	a, (0x04, sp)
	push	a
	call	_SPI_transfer
	pop	a
;	main.c: 57: while(size--) {
	ldw	x, (0x05, sp)
	ld	a, (0x07, sp)
	ld	(0x01, sp), a
00101$:
	ld	a, (0x01, sp)
	dec	(0x01, sp)
	tnz	a
	jreq	00103$
;	main.c: 58: *(buffer++) = SPI_transfer(0xff);
	pushw	x
	push	#0xff
	call	_SPI_transfer
	addw	sp, #1
	popw	x
	ld	(x), a
	incw	x
	jra	00101$
00103$:
;	main.c: 60: SPI_chip_deselect();
	pop	a
	jp	_SPI_chip_deselect
;	main.c: 61: }
	pop	a
	ret
;	main.c: 62: void flush_tx() {
;	-----------------------------------------
;	 function flush_tx
;	-----------------------------------------
_flush_tx:
;	main.c: 63: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 64: SPI_transfer(FLUSH_TX);
	push	#0xe1
	call	_SPI_transfer
	pop	a
;	main.c: 65: SPI_chip_deselect();
;	main.c: 66: }
	jp	_SPI_chip_deselect
;	main.c: 67: void flush_rx() {
;	-----------------------------------------
;	 function flush_rx
;	-----------------------------------------
_flush_rx:
;	main.c: 68: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 69: SPI_transfer(FLUSH_RX);
	push	#0xe2
	call	_SPI_transfer
	pop	a
;	main.c: 70: SPI_chip_deselect();
;	main.c: 71: }
	jp	_SPI_chip_deselect
;	main.c: 72: uint8_t get_status () {
;	-----------------------------------------
;	 function get_status
;	-----------------------------------------
_get_status:
;	main.c: 73: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 74: SPI_transfer(R_REGISTER | (REGISTER_MASK & STATUS_NRF));
	push	#0x07
	call	_SPI_transfer
	pop	a
;	main.c: 75: uint8_t ret = SPI_transfer(0xff);
	push	#0xff
	call	_SPI_transfer
	addw	sp, #1
;	main.c: 76: SPI_chip_deselect();
	push	a
	call	_SPI_chip_deselect
	pop	a
;	main.c: 77: return ret;
;	main.c: 78: }
	ret
;	main.c: 79: void clear_status () {
;	-----------------------------------------
;	 function clear_status
;	-----------------------------------------
_clear_status:
;	main.c: 80: SPI_chip_select();
	call	_SPI_chip_select
;	main.c: 81: SPI_transfer(W_REGISTER | (REGISTER_MASK & STATUS_NRF));
	push	#0x27
	call	_SPI_transfer
	pop	a
;	main.c: 82: SPI_transfer(_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	push	#0x70
	call	_SPI_transfer
	pop	a
;	main.c: 83: SPI_chip_deselect();
;	main.c: 84: }
	jp	_SPI_chip_deselect
;	main.c: 86: int putchar(int c) {
;	-----------------------------------------
;	 function putchar
;	-----------------------------------------
_putchar:
;	main.c: 87: uart_write(c);
	ld	a, (0x04, sp)
	push	a
	call	_uart_write
	pop	a
;	main.c: 88: return 0;
	clrw	x
;	main.c: 89: }
	ret
;	main.c: 91: int main () {
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	sub	sp, #12
;	main.c: 92: CLK_CKDIVR = 0;//16mhz
	mov	0x50c6+0, #0x00
;	main.c: 95: uart_init(9600);
	push	#0x80
	push	#0x25
	call	_uart_init
	addw	sp, #2
;	main.c: 97: PC_DDR |= _BV(CSN_PIN);
	bset	20492, #4
;	main.c: 98: PC_CR1 |= _BV(CSN_PIN);
	bset	20493, #4
;	main.c: 99: PC_ODR |= _BV(CSN_PIN);
	bset	20490, #4
;	main.c: 104: SPI_init();
	call	_SPI_init
;	main.c: 106: while(1) {
00104$:
;	main.c: 107: printf("Initting radio.\n\r");
	push	#<(___str_0 + 0)
	push	#((___str_0 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 109: delay_milliseconds(100); //OFF_TO_POWERDOWN_MILLIS
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 111: uint8_t channel = 100;
	ld	a, #0x64
	ld	(0x01, sp), a
;	main.c: 112: write_register_block(RF_CH, &channel, 1);
	ldw	x, sp
	incw	x
	push	#0x01
	pushw	x
	push	#0x05
	call	_write_register_block
	addw	sp, #4
;	main.c: 114: uint8_t rf_setup = RF_TX_PWR_MAX | _BV(3);
	ld	a, #0x0e
	ld	(0x02, sp), a
;	main.c: 115: write_register_block(RF_SETUP, &rf_setup, 1);
	ldw	x, sp
	incw	x
	incw	x
	push	#0x01
	pushw	x
	push	#0x06
	call	_write_register_block
	addw	sp, #4
;	main.c: 117: uint8_t address[] = {7,7,7,7,7};
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
;	main.c: 118: write_register_block(RX_ADDR_P1, address, 5);
	push	#0x05
	ldw	x, sp
	addw	x, #4
	pushw	x
	push	#0x0b
	call	_write_register_block
	addw	sp, #4
;	main.c: 120: uint8_t dynpd = _BV(DPL_P0) | _BV(DPL_P1);
	ld	a, #0x03
	ld	(0x08, sp), a
;	main.c: 121: write_register_block(DYNPD, &dynpd, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #9
	pushw	x
	push	#0x1c
	call	_write_register_block
	addw	sp, #4
;	main.c: 123: uint8_t feature = _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK);
	ld	a, #0x07
	ld	(0x09, sp), a
;	main.c: 124: write_register_block(FEATURE, &feature, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #10
	pushw	x
	push	#0x1d
	call	_write_register_block
	addw	sp, #4
;	main.c: 126: flush_rx();
	call	_flush_rx
;	main.c: 127: flush_tx();
	call	_flush_tx
;	main.c: 129: clear_status();
	call	_clear_status
;	main.c: 131: uint8_t config = CONFIG_REG_SETTINGS_FOR_RX_MODE;
	ld	a, #0x0b
	ld	(0x0a, sp), a
;	main.c: 132: write_register_block(CONFIG, &config, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #11
	pushw	x
	push	#0x00
	call	_write_register_block
	addw	sp, #4
;	main.c: 134: delay_milliseconds(5);
	push	#0x05
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 136: uint8_t config_check = 0;
	clr	(0x0b, sp)
;	main.c: 137: read_register_block(CONFIG, &config_check, 1);
	push	#0x01
	ldw	x, sp
	addw	x, #12
	pushw	x
	push	#0x00
	call	_read_register_block
	addw	sp, #4
;	main.c: 139: if(config_check == CONFIG_REG_SETTINGS_FOR_RX_MODE) {
	ld	a, (0x0b, sp)
	cp	a, #0x0b
	jreq	00158$
	jp	00104$
00158$:
;	main.c: 144: printf("Init successufull!!!\n\r");
	push	#<(___str_1 + 0)
	push	#((___str_1 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 146: while(1) {
00113$:
;	main.c: 147: uint8_t status = get_status();
	call	_get_status
	ld	(0x0c, sp), a
;	main.c: 149: printf("status: "); printbin(status);
	push	#<(___str_2 + 0)
	push	#((___str_2 + 0) >> 8)
	call	_printf
	addw	sp, #2
	ld	a, (0x0c, sp)
	push	a
	call	_printbin
	pop	a
;	main.c: 150: printf("\n\r");
	push	#<(___str_3 + 0)
	push	#((___str_3 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 152: if((status & 0b1110) == 0b1110) {//no data
	ld	a, (0x0c, sp)
	and	a, #0x0e
	ld	xl, a
	clr	a
	ld	xh, a
	cpw	x, #0x000e
	jrne	00110$
;	main.c: 154: printf("No sig.\n\r");
	push	#<(___str_4 + 0)
	push	#((___str_4 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 156: delay_milliseconds(40);
	push	#0x28
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 158: continue;
	jra	00113$
00110$:
;	main.c: 161: read_payload_data(R_RX_PAYLOAD, payload_buffer, PAYLOAD_SIZE);
	push	#0x05
	push	#<(_payload_buffer + 0)
	push	#((_payload_buffer + 0) >> 8)
	push	#0x61
	call	_read_payload_data
	addw	sp, #4
;	main.c: 163: if(get_status() & _BV(RX_DR)) {
	call	_get_status
	bcp	a, #0x40
	jreq	00107$
;	main.c: 164: clear_status();
	call	_clear_status
00107$:
;	main.c: 167: printf("data: ");
	push	#<(___str_5 + 0)
	push	#((___str_5 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 168: for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
	clr	(0x0c, sp)
00116$:
	ld	a, (0x0c, sp)
	cp	a, #0x05
	jrnc	00108$
;	main.c: 169: printf("%u, ", payload_buffer[i]);
	clrw	x
	ld	a, (0x0c, sp)
	ld	xl, a
	addw	x, #(_payload_buffer + 0)
	ld	a, (x)
	clrw	x
	ld	xl, a
	pushw	x
	push	#<(___str_6 + 0)
	push	#((___str_6 + 0) >> 8)
	call	_printf
	addw	sp, #4
;	main.c: 168: for(uint8_t i = 0; i < PAYLOAD_SIZE; i++) {
	inc	(0x0c, sp)
	jra	00116$
00108$:
;	main.c: 171: printf("\n\r");
	push	#<(___str_3 + 0)
	push	#((___str_3 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	main.c: 173: delay_milliseconds(1);
	push	#0x01
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	main.c: 176: }
	jp	00113$
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
	.ascii "status: "
	.db 0x00
	.area CODE
	.area CONST
___str_3:
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_4:
	.ascii "No sig."
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_5:
	.ascii "data: "
	.db 0x00
	.area CODE
	.area CONST
___str_6:
	.ascii "%u, "
	.db 0x00
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
