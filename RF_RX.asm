;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 4.0.0 #11528 (Linux)
;--------------------------------------------------------
	.module RF_RX
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _startRx
	.globl _readRegisterBlock
	.globl _writeRegisterBlock
	.globl _SPIReadBlock
	.globl _SPIWriteBlock
	.globl _SPIRead
	.globl _SPIWrite
	.globl _SPI_read
	.globl _SPI_write
	.globl _printf
	.globl _ce
	.globl _csn
	.globl _delay_milliseconds
	.globl _preInit
	.globl _writeRegister
	.globl _readRegister
	.globl _initRadio
	.globl _hasData
	.globl _readData
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
;	RF_RX.c: 7: void ce(uint8_t state) {
;	-----------------------------------------
;	 function ce
;	-----------------------------------------
_ce:
;	RF_RX.c: 9: PC_ODR |= _BV(CE_PIN);
	ld	a, 0x500a
;	RF_RX.c: 8: if(state) {
	tnz	(0x03, sp)
	jreq	00102$
;	RF_RX.c: 9: PC_ODR |= _BV(CE_PIN);
	or	a, #0x08
	ld	0x500a, a
	ret
00102$:
;	RF_RX.c: 12: PC_ODR &= ~_BV(CE_PIN);
	and	a, #0xf7
	ld	0x500a, a
;	RF_RX.c: 14: }
	ret
;	RF_RX.c: 15: void csn(uint8_t state) {
;	-----------------------------------------
;	 function csn
;	-----------------------------------------
_csn:
;	RF_RX.c: 17: PC_ODR |= _BV(CSN_PIN);
	ld	a, 0x500a
;	RF_RX.c: 16: if(state) {
	tnz	(0x03, sp)
	jreq	00102$
;	RF_RX.c: 17: PC_ODR |= _BV(CSN_PIN);
	or	a, #0x10
	ld	0x500a, a
	ret
00102$:
;	RF_RX.c: 20: PC_ODR &= ~_BV(CSN_PIN);
	and	a, #0xef
	ld	0x500a, a
;	RF_RX.c: 22: }
	ret
;	RF_RX.c: 24: void SPIWrite (uint8_t data) {
;	-----------------------------------------
;	 function SPIWrite
;	-----------------------------------------
_SPIWrite:
;	RF_RX.c: 27: SPI_write(data);
	ld	a, (0x03, sp)
	push	a
	call	_SPI_write
	pop	a
;	RF_RX.c: 28: }
	ret
;	RF_RX.c: 29: uint8_t SPIRead() {
;	-----------------------------------------
;	 function SPIRead
;	-----------------------------------------
_SPIRead:
;	RF_RX.c: 31: return SPI_read();
;	RF_RX.c: 32: }
	jp	_SPI_read
;	RF_RX.c: 34: void delay_milliseconds(unsigned long milliseconds) {
;	-----------------------------------------
;	 function delay_milliseconds
;	-----------------------------------------
_delay_milliseconds:
	push	a
;	RF_RX.c: 36: for(uint8_t m = 0; m < milliseconds; m++) {
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
;	RF_RX.c: 37: for(uint8_t i = 0; i < 160; i++) {
	clrw	x
00108$:
	ld	a, xl
	cp	a, #0xa0
	jrnc	00112$
;	RF_RX.c: 38: for(uint8_t j = 0; j < 10; j++) {
	clr	a
00105$:
	cp	a, #0x0a
	jrnc	00109$
;	RF_RX.c: 39: __asm__("nop");
	nop
;	RF_RX.c: 40: __asm__("nop");
	nop
;	RF_RX.c: 41: __asm__("nop");
	nop
;	RF_RX.c: 42: __asm__("nop");
	nop
;	RF_RX.c: 38: for(uint8_t j = 0; j < 10; j++) {
	inc	a
	jra	00105$
00109$:
;	RF_RX.c: 37: for(uint8_t i = 0; i < 160; i++) {
	incw	x
	jra	00108$
00112$:
;	RF_RX.c: 36: for(uint8_t m = 0; m < milliseconds; m++) {
	inc	(0x01, sp)
	jra	00111$
00113$:
;	RF_RX.c: 46: }
	pop	a
	ret
;	RF_RX.c: 48: void preInit (void * spi_handle) {
;	-----------------------------------------
;	 function preInit
;	-----------------------------------------
_preInit:
;	RF_RX.c: 50: }
	ret
;	RF_RX.c: 54: void SPIWriteBlock(void * data, uint8_t size) {
;	-----------------------------------------
;	 function SPIWriteBlock
;	-----------------------------------------
_SPIWriteBlock:
	push	a
;	RF_RX.c: 55: for(uint8_t i = 0; i < size; i++)
	clr	(0x01, sp)
00103$:
	ld	a, (0x01, sp)
	cp	a, (0x06, sp)
	jrnc	00105$
;	RF_RX.c: 57: SPIWrite(*((uint8_t *)data+i));
	ldw	x, (0x04, sp)
	ld	a, xl
	add	a, (0x01, sp)
	rlwa	x
	adc	a, #0x00
	ld	xh, a
	ld	a, (x)
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 55: for(uint8_t i = 0; i < size; i++)
	inc	(0x01, sp)
	jra	00103$
00105$:
;	RF_RX.c: 59: }
	pop	a
	ret
;	RF_RX.c: 60: void SPIReadBlock(void * buff, uint8_t size) {
;	-----------------------------------------
;	 function SPIReadBlock
;	-----------------------------------------
_SPIReadBlock:
	push	a
;	RF_RX.c: 61: for(uint8_t i = 0; i < size; i++)
	clr	(0x01, sp)
00103$:
	ld	a, (0x01, sp)
	cp	a, (0x06, sp)
	jrnc	00105$
;	RF_RX.c: 63: *(uint8_t *)((uint8_t *)buff+i) = SPIRead();
	ldw	x, (0x04, sp)
	ld	a, xl
	add	a, (0x01, sp)
	rlwa	x
	adc	a, #0x00
	ld	xh, a
	pushw	x
	call	_SPIRead
	popw	x
	ld	(x), a
;	RF_RX.c: 61: for(uint8_t i = 0; i < size; i++)
	inc	(0x01, sp)
	jra	00103$
00105$:
;	RF_RX.c: 65: }
	pop	a
	ret
;	RF_RX.c: 66: void writeRegister(uint8_t reg, uint8_t data) {
;	-----------------------------------------
;	 function writeRegister
;	-----------------------------------------
_writeRegister:
	sub	sp, #2
;	RF_RX.c: 67: printf("writing %02x to reg: %02x\n\r", data, reg);
	clrw	x
	ld	a, (0x05, sp)
	ld	xl, a
	ld	a, (0x06, sp)
	clr	(0x01, sp)
	pushw	x
	push	a
	ld	a, (0x04, sp)
	push	a
	push	#<(___str_0 + 0)
	push	#((___str_0 + 0) >> 8)
	call	_printf
	addw	sp, #6
;	RF_RX.c: 68: csn(0);
	push	#0x00
	call	_csn
	pop	a
;	RF_RX.c: 69: SPIWrite(W_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x05, sp)
	and	a, #0x1f
	or	a, #0x20
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 70: SPIWrite(data);
	ld	a, (0x06, sp)
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 71: csn(1);
	push	#0x01
	call	_csn
;	RF_RX.c: 72: }
	addw	sp, #3
	ret
;	RF_RX.c: 73: void writeRegisterBlock(uint8_t reg, void * data, uint8_t size) {
;	-----------------------------------------
;	 function writeRegisterBlock
;	-----------------------------------------
_writeRegisterBlock:
;	RF_RX.c: 74: csn(0);
	push	#0x00
	call	_csn
	pop	a
;	RF_RX.c: 75: SPIWrite(W_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x03, sp)
	and	a, #0x1f
	or	a, #0x20
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 76: SPIWriteBlock(data, size);
	ld	a, (0x06, sp)
	push	a
	ldw	x, (0x05, sp)
	pushw	x
	call	_SPIWriteBlock
	addw	sp, #3
;	RF_RX.c: 77: csn(1);
	push	#0x01
	call	_csn
	pop	a
;	RF_RX.c: 78: }
	ret
;	RF_RX.c: 79: uint8_t readRegister(uint8_t reg) {
;	-----------------------------------------
;	 function readRegister
;	-----------------------------------------
_readRegister:
	sub	sp, #3
;	RF_RX.c: 80: csn(0);
	push	#0x00
	call	_csn
	pop	a
;	RF_RX.c: 81: SPIWrite(R_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x06, sp)
	ld	(0x03, sp), a
	and	a, #0x1f
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 82: uint8_t ret_data = SPIRead();
	call	_SPIRead
	ld	(0x01, sp), a
;	RF_RX.c: 83: csn(1);
	push	#0x01
	call	_csn
	pop	a
;	RF_RX.c: 84: printf("read %02x from reg: %02x\n\r", ret_data, reg);
	clrw	x
	ld	a, (0x03, sp)
	ld	xl, a
	ld	a, (0x01, sp)
	clr	(0x02, sp)
	pushw	x
	push	a
	ld	a, (0x05, sp)
	push	a
	push	#<(___str_1 + 0)
	push	#((___str_1 + 0) >> 8)
	call	_printf
	addw	sp, #6
;	RF_RX.c: 85: return ret_data;
	ld	a, (0x01, sp)
;	RF_RX.c: 86: }
	addw	sp, #3
	ret
;	RF_RX.c: 87: void readRegisterBlock(uint8_t reg, void * buff, uint8_t size) {
;	-----------------------------------------
;	 function readRegisterBlock
;	-----------------------------------------
_readRegisterBlock:
;	RF_RX.c: 88: csn(0);
	push	#0x00
	call	_csn
	pop	a
;	RF_RX.c: 89: SPIWrite(R_REGISTER | (REGISTER_MASK & reg));
	ld	a, (0x03, sp)
	and	a, #0x1f
	push	a
	call	_SPIWrite
	pop	a
;	RF_RX.c: 90: SPIReadBlock(buff, size);
	ld	a, (0x06, sp)
	push	a
	ldw	x, (0x05, sp)
	pushw	x
	call	_SPIReadBlock
	addw	sp, #3
;	RF_RX.c: 91: csn(1);
	push	#0x01
	call	_csn
	pop	a
;	RF_RX.c: 92: }
	ret
;	RF_RX.c: 96: uint8_t startRx() {
;	-----------------------------------------
;	 function startRx
;	-----------------------------------------
_startRx:
;	RF_RX.c: 98: flushTx();
	push	#0xff
	push	#0xe1
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 99: resetStatus();
	push	#0x70
	push	#0x07
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 101: printf("seting up config, ");
	push	#<(___str_2 + 0)
	push	#((___str_2 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 102: ce(0); // Put radio into Standby-I mode in order to transition into RX mode.
	push	#0x00
	call	_ce
	pop	a
;	RF_RX.c: 103: writeRegister(CONFIG, CONFIG_REG_SETTINGS_FOR_RX_MODE);
	push	#0x0b
	push	#0x00
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 104: ce(1);
	push	#0x01
	call	_ce
	pop	a
;	RF_RX.c: 107: printf("delay, ");
	push	#<(___str_3 + 0)
	push	#((___str_3 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 108: delay_milliseconds(POWERDOWN_TO_RXTX_MODE_MILLIS);
	push	#0x05
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	RF_RX.c: 110: uint8_t config_value = readRegister(CONFIG);
	push	#0x00
	call	_readRegister
	addw	sp, #1
;	RF_RX.c: 111: printf("reading config -> result: %02x, ", config_value);
	clrw	x
	ld	xl, a
	push	a
	pushw	x
	push	#<(___str_4 + 0)
	push	#((___str_4 + 0) >> 8)
	call	_printf
	addw	sp, #4
	pop	a
;	RF_RX.c: 112: return config_value == CONFIG_REG_SETTINGS_FOR_RX_MODE;
	sub	a, #0x0b
	jrne	00104$
	inc	a
	ret
00104$:
	clr	a
;	RF_RX.c: 113: }
	ret
;	RF_RX.c: 114: uint8_t initRadio (uint8_t * receive_address, uint8_t bitrate, uint8_t channel) {
;	-----------------------------------------
;	 function initRadio
;	-----------------------------------------
_initRadio:
;	RF_RX.c: 117: printf("initRadio, ");
	push	#<(___str_5 + 0)
	push	#((___str_5 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 118: csn(1);
	push	#0x01
	call	_csn
	pop	a
;	RF_RX.c: 120: printf("csn hi, delay, ");
	push	#<(___str_6 + 0)
	push	#((___str_6 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 121: delay_milliseconds(OFF_TO_POWERDOWN_MILLIS);
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_milliseconds
	addw	sp, #4
;	RF_RX.c: 123: printf("set channel: ");
	push	#<(___str_7 + 0)
	push	#((___str_7 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 124: setChannel(channel);
	ld	a, (0x06, sp)
	push	a
	push	#0x05
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 126: printf("Writing to fr_setup: ");
	push	#<(___str_8 + 0)
	push	#((___str_8 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 127: writeRegister(RF_SETUP, bitrate | RF_TX_PWR_MAX);
	ld	a, (0x05, sp)
	or	a, #0x06
	push	a
	push	#0x06
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 129: if(receive_address != NULL)
;	RF_RX.c: 130: writeRegisterBlock(RX_ADDR_P1, receive_address, 5);
	ldw	x, (0x03, sp)
	jreq	00102$
	push	#0x05
	pushw	x
	push	#0x0b
	call	_writeRegisterBlock
	addw	sp, #4
00102$:
;	RF_RX.c: 133: printf("writing to feature regs, ");
	push	#<(___str_9 + 0)
	push	#((___str_9 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 134: writeRegister(DYNPD, _BV(DPL_P0) | _BV(DPL_P1));
	push	#0x03
	push	#0x1c
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 135: writeRegister(FEATURE, _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));
	push	#0x07
	push	#0x1d
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 137: printf("flushing, ");
	push	#<(___str_10 + 0)
	push	#((___str_10 + 0) >> 8)
	call	_printf
	addw	sp, #2
;	RF_RX.c: 138: flushRx();
	push	#0xff
	push	#0xe2
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 139: flushTx();
	push	#0xff
	push	#0xe1
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 140: resetStatus();
	push	#0x70
	push	#0x07
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 142: return startRx();
;	RF_RX.c: 143: }
	jp	_startRx
;	RF_RX.c: 144: uint8_t hasData() {
;	-----------------------------------------
;	 function hasData
;	-----------------------------------------
_hasData:
;	RF_RX.c: 145: return ((getStatus()>>1) & 0b111) != 0b111; //rx fifo not empty
	push	#0x07
	call	_readRegister
	addw	sp, #1
	srl	a
	and	a, #0x07
	ld	xl, a
	clr	a
	ld	xh, a
	cpw	x, #0x0007
	jrne	00104$
	ld	a, #0x01
	.byte 0x21
00104$:
	clr	a
00105$:
	xor	a, #0x01
;	RF_RX.c: 146: }
	ret
;	RF_RX.c: 147: void readData (void * data, uint8_t size) {
;	-----------------------------------------
;	 function readData
;	-----------------------------------------
_readData:
;	RF_RX.c: 148: if(!size)
	tnz	(0x05, sp)
	jrne	00102$
;	RF_RX.c: 149: size = readRegister(R_RX_PL_WID); //auto length
	push	#0x60
	call	_readRegister
	addw	sp, #1
	ld	(0x05, sp), a
00102$:
;	RF_RX.c: 151: csn(0);
	push	#0x00
	call	_csn
	pop	a
;	RF_RX.c: 152: SPIWrite(R_RX_PAYLOAD);
	push	#0x61
	call	_SPIWrite
	pop	a
;	RF_RX.c: 153: SPIReadBlock(data, size);
	ld	a, (0x05, sp)
	push	a
	ldw	x, (0x04, sp)
	pushw	x
	call	_SPIReadBlock
	addw	sp, #3
;	RF_RX.c: 154: csn(1);
	push	#0x01
	call	_csn
	pop	a
;	RF_RX.c: 156: uint8_t status_reg = getStatus();
	push	#0x07
	call	_readRegister
	addw	sp, #1
;	RF_RX.c: 157: if(status_reg & _BV(RX_DR))
	bcp	a, #0x40
	jrne	00118$
	ret
00118$:
;	RF_RX.c: 159: resetStatus();
	push	#0x70
	push	#0x07
	call	_writeRegister
	addw	sp, #2
;	RF_RX.c: 161: }
	ret
	.area CODE
	.area CONST
	.area CONST
___str_0:
	.ascii "writing %02x to reg: %02x"
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_1:
	.ascii "read %02x from reg: %02x"
	.db 0x0a
	.db 0x0d
	.db 0x00
	.area CODE
	.area CONST
___str_2:
	.ascii "seting up config, "
	.db 0x00
	.area CODE
	.area CONST
___str_3:
	.ascii "delay, "
	.db 0x00
	.area CODE
	.area CONST
___str_4:
	.ascii "reading config -> result: %02x, "
	.db 0x00
	.area CODE
	.area CONST
___str_5:
	.ascii "initRadio, "
	.db 0x00
	.area CODE
	.area CONST
___str_6:
	.ascii "csn hi, delay, "
	.db 0x00
	.area CODE
	.area CONST
___str_7:
	.ascii "set channel: "
	.db 0x00
	.area CODE
	.area CONST
___str_8:
	.ascii "Writing to fr_setup: "
	.db 0x00
	.area CODE
	.area CONST
___str_9:
	.ascii "writing to feature regs, "
	.db 0x00
	.area CODE
	.area CONST
___str_10:
	.ascii "flushing, "
	.db 0x00
	.area CODE
	.area INITIALIZER
	.area CABS (ABS)
