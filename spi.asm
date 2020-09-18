;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.0.0 #11528 (Linux)
;--------------------------------------------------------
	.module spi
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _SPI_chip_select
	.globl _SPI_chip_deselect
	.globl _SPI_init
	.globl _SPI_write
	.globl _SPI_read
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
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
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	spi.c: 4: void SPI_chip_select() {
;	-----------------------------------------
;	 function SPI_chip_select
;	-----------------------------------------
_SPI_chip_select:
;	spi.c: 5: PC_ODR &= ~(1 << SPI_CS_PIN);
	bres	20490, #4
;	spi.c: 6: }
	ret
;	spi.c: 7: void SPI_chip_deselect() {
;	-----------------------------------------
;	 function SPI_chip_deselect
;	-----------------------------------------
_SPI_chip_deselect:
;	spi.c: 8: while ((SPI_SR & (1 << SPI_SR_BSY)));
00101$:
	ld	a, 0x5203
	jrmi	00101$
;	spi.c: 9: PC_ODR |= (1 << SPI_CS_PIN);
	bset	20490, #4
;	spi.c: 10: }
	ret
;	spi.c: 12: void SPI_init() {
;	-----------------------------------------
;	 function SPI_init
;	-----------------------------------------
_SPI_init:
;	spi.c: 14: PC_DDR |= (1 << SPI_CS_PIN);
	bset	20492, #4
;	spi.c: 15: PC_CR1 |= (1 << SPI_CS_PIN);
	bset	20493, #4
;	spi.c: 16: PC_ODR |= (1 << SPI_CS_PIN);
	bset	20490, #4
;	spi.c: 18: SPI_CR2 = (1 << SPI_CR2_SSM) | (1 << SPI_CR2_SSI);
	mov	0x5201+0, #0x03
;	spi.c: 19: SPI_CR1 = (1 << SPI_CR1_MSTR) | (1 << SPI_CR1_SPE) | (1 << SPI_CR1_BR0);
	mov	0x5200+0, #0x4c
;	spi.c: 20: }
	ret
;	spi.c: 22: void SPI_write(uint8_t data) {
;	-----------------------------------------
;	 function SPI_write
;	-----------------------------------------
_SPI_write:
;	spi.c: 23: SPI_DR = data;
	ldw	x, #0x5204
	ld	a, (0x03, sp)
	ld	(x), a
;	spi.c: 24: while (!(SPI_SR & (1 << SPI_SR_TXE)));
00101$:
	ld	a, 0x5203
	bcp	a, #0x02
	jreq	00101$
;	spi.c: 25: }
	ret
;	spi.c: 26: uint8_t SPI_read() {
;	-----------------------------------------
;	 function SPI_read
;	-----------------------------------------
_SPI_read:
;	spi.c: 28: SPI_write(0xff);
	push	#0xff
	call	_SPI_write
	pop	a
;	spi.c: 29: while (!(SPI_SR & (1 << SPI_SR_RXNE)));
00101$:
	ld	a, 0x5203
	srl	a
	jrnc	00101$
;	spi.c: 30: return SPI_DR;
	ld	a, 0x5204
;	spi.c: 31: }
	ret
	.area CODE
	.area CONST
	.area INITIALIZER
	.area CABS (ABS)
