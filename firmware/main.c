/* 
 * Copyright (C) 2010 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <string.h>
#include <LUFA/Version.h>
#include <LUFA/Drivers/USB/USB.h>
#include "desc.h"
#include "usart.h"
#include "parse.h"
#include "types.h"
#include "../commands.h"

//#define DEBUG 1
#define RETRIES 3
#define CHUNK_SIZE 64

static uint32 m_status = 0x00000000;
static uint32 m_idleCycles;
#ifndef RETRIES
	static uint8  m_repeats;
#endif
static uint32 m_failures;

int main(void) {
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	PORTB = 0x00;
	DDRB = 0x00;
	usartInit();
	usartSendFlashString(PSTR("NanduinoJTAG...\n"));
	USB_Init();
	
	for ( ; ; ) {
		USB_USBTask();
	}
}

void EVENT_USB_Device_Connect(void) {
	// Connected
}

void EVENT_USB_Device_Disconnect(void) {
	// Disconnected
}

void EVENT_USB_Device_ConfigurationChanged(void) {
	m_status = 0x00000000;
	if ( !(Endpoint_ConfigureEndpoint(IN_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_IN,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		m_status |= 0xDEAD0000;
	}
	if ( !(Endpoint_ConfigureEndpoint(OUT_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_OUT,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		m_status |= 0x0000DEAD;
	}
}

// Bit masks on Port B for the four JTAG lines
#define TCK 0x80
#define TMS 0x40
#define TDO 0x20
#define TDI 0x10

// JTAG instructions
#define INS_PROG_ENABLE   0x04
#define INS_PROG_COMMANDS 0x05
#define INS_PROG_PAGELOAD 0x06
#define INS_PROG_PAGEREAD 0x07
#define INS_AVR_RESET     0x0C

// AVR Commands
#define CMD_LOAD_DATA_HIGH_BYTE    0x1700
#define CMD_LOAD_DATA_LOW_BYTE     0x1300
#define CMD_LOAD_ADDRESS_HIGH_BYTE 0x0700
#define CMD_LOAD_ADDRESS_LOW_BYTE  0x0300

#define CMD_1A_CHIP_ERASE_1      0x2380
#define CMD_1A_CHIP_ERASE_2      0x3180
#define CMD_1A_CHIP_ERASE_3      0x3380
#define CMD_1A_POLL_ERASE        0x3380
#define CMD_3A_ENTER_FLASH_READ  0x2302
#define CMD_2A_ENTER_FLASH_WRITE 0x2310
#define CMD_2G_WRITE_FLASH_PAGE  0x3700
#define CMD_2H_POLL_FLASH_PAGE   0x3700

#define CMD_6A_ENTER_FUSE_WRITE  0x2340
#define CMD_6C_WRITE_EXT_BYTE    0x3B00
#define CMD_6D_POLL_EXT_BYTE     0x3700
#define CMD_6F_WRITE_HIGH_BYTE   0x3700
#define CMD_6G_POLL_HIGH_BYTE    0x3700
#define CMD_6I_WRITE_LOW_BYTE    0x3300
#define CMD_6J_POLL_LOW_BYTE     0x3300
#define CMD_7A_ENTER_LOCK_WRITE  0x2320
#define CMD_7C_WRITE_LOCK_BYTE   0x3300
#define CMD_7D_POLL_LOCK_BYTE    0x3300
#define CMD_8A_ENTER_FUSE_READ   0x2304
#define CMD_8F_READ_FUSES        0x3A00
#define CMD_8F_READ_EXT_BYTE     0x3E00
#define CMD_8F_READ_HIGH_BYTE    0x3200
#define CMD_8F_READ_LOW_BYTE     0x3600
#define CMD_8F_READ_LOCK_BITS    0x3700

// Execute one TCK cycle of the JTAG TAP state machine
//
static inline uint8 jtagClock(uint8 input) {
	uint8 value = PORTB;
	value &= ~(TCK|TMS|TDI);
	input &= (TMS|TDI);
	value |= input;
	PORTB = value;
	PORTB = value | TCK;
	PORTB = value;
	return PINB & TDO;
}

// Shortcut function to navigate quickly to Shift-DR (from Run-Test/Idle) and
// Shift-IR (from Select-DR Scan).
//
static inline void jtagGotoShiftState(void) {
	jtagClock(TMS);  // Now in Select-xR Scan
	jtagClock(0);    // Now in Capture-xR
	jtagClock(0);    // Now in Shift-xR
}

// After writing data or instruction
//
static inline void jtagGotoIdleState(void) {
	jtagClock(TMS);      // Now in Update-DR
	jtagClock(0);        // Now in Run-Test/Idle
}

// Write a byte and read back a byte; stay in Shift-DR
//
uint8 jtagExchangeData(uint8 data) {
	uint8 result = 0x00;
	uint8 i;
	for ( i = 0; i < 7; i++ ) {
		if ( jtagClock(data&0x01 ? TDI : 0) ) {
			result |= 0x80;
		}
		result >>= 1;
		data >>= 1;
	}
	if ( jtagClock(data&0x01 ? TDI : 0) ) {  // Still in Shift-DR
		result |= 0x80;
	}
	return result;
}

// Write a byte and read back a byte; exit to Exit1-DR
//
uint8 jtagExchangeDataEnd(uint8 data) {
	uint8 result = 0x00;
	uint8 i;
	for ( i = 0; i < 7; i++ ) {
		if ( jtagClock(data&0x01 ? TDI : 0) ) {
			result |= 0x80;
		}
		result >>= 1;
		data >>= 1;
	}
	if ( jtagClock(data&0x01 ? TDI|TMS : TMS) ) {  // Now in Exit1-DR
		result |= 0x80;
	}
	return result;
}

// Write numBits bits from the supplied uint8 and read back numBits bits
//
uint8 jtagExchangeData8(uint8 data, uint8 numBits) {
	const uint8 extraShift = 8-numBits;
	uint8 result = 0x00;
	numBits--;
	while ( numBits ) {
		if ( jtagClock(data&0x01 ? TDI : 0) ) {
			result |= 0x80;
		}
		result >>= 1;
		data >>= 1;
		numBits--;
	}
	if ( jtagClock(data&0x01 ? TDI|TMS : TMS) ) {  // Now in Exit1-DR
		result |= 0x80;
	}
	result >>= extraShift;
	return result;
}

// Write numBits bits from the supplied uint16 and read back numBits bits
//
uint16 jtagExchangeData16(uint16 data, uint8 numBits) {
	const uint8 extraShift = 16-numBits;
	uint16 result = 0x0000;
	numBits--;
	while ( numBits ) {
		if ( jtagClock(data&0x0001 ? TDI : 0) ) {
			result |= 0x8000;
		}
		result >>= 1;
		data >>= 1;
		numBits--;
	}
	if ( jtagClock(data&0x0001 ? TDI|TMS : TMS) ) {  // Now in Exit1-DR
		result |= 0x8000;
	}
	result >>= extraShift;
	return result;
}

// Write numBits bits from the supplied uint32 and read back numBits bits
//
uint32 jtagExchangeData32(uint32 data, uint8 numBits) {
	const uint8 extraShift = 32-numBits;
	uint32 result = 0x00000000;
	numBits--;
	while ( numBits ) {
		if ( jtagClock(data&0x00000001 ? TDI : 0) ) {
			result |= 0x80000000;
		}
		result >>= 1;
		data >>= 1;
		numBits--;
	}
	if ( jtagClock(data&0x00000001 ? TDI|TMS : TMS) ) {  // Now in Exit1-DR
		result |= 0x80000000;
	}
	result >>= extraShift;
	return result;
}

// Write the specified 4-bit JTAG instruction
//
void jtagWriteInstruction(uint8 cmd) {
	jtagClock(TMS);      // Now in Select-DR Scan
	jtagGotoShiftState();  // Now in Shift-IR
	jtagExchangeData8(cmd, 4);  // Now in Exit1-IR
	jtagGotoIdleState();          // Now in Run-Test/Idle
}

// Reset the JTAG TAP state machine and return the IDENT register
//
uint32 jtagResetAndGetIdentRegister(void) {
	uint32 idCode;

	// Go to Test-Logic-Reset
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);

	// Go to Run-Test/Idle
	jtagClock(0);        // Now in Run-Test/Idle

	// Go to Shift-DR
	jtagGotoShiftState();  // Now in Shift-DR
	idCode = jtagExchangeData32(0x00000000, 32);  // Now in Exit1-DR
	jtagGotoIdleState();          // Now in Run-Test/Idle
	return idCode;
}

// Reset the JTAG TAP state machine
//
void jtagReset(void) {
	// Go to Test-Logic-Reset
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);
	jtagClock(TMS);
}

// Set the RESET state of the device
//
void avrResetEnable(uint8 enable) {
	jtagWriteInstruction(INS_AVR_RESET);
	jtagGotoShiftState();    // Now in Shift-DR
	if ( enable ) {
		jtagClock(TDI|TMS);  // Now in Exit1-DR
	} else {
		jtagClock(TMS);      // Now in Exit1-DR
	}
	jtagGotoIdleState();          // Now in Run-Test/Idle
}

// Enable/disable the programming mode by writing a magic value
//
void avrProgModeEnable(uint8 enable) {
	jtagWriteInstruction(INS_PROG_ENABLE);
	jtagGotoShiftState();  // Now in Shift-DR
	if ( enable ) {
		jtagExchangeData16(0xA370, 16);  // Magic word! Now in Exit1-DR
	} else {
		jtagExchangeData16(0x0000, 16);  // Now in Exit1-DR
	}
	jtagGotoIdleState();        // Now in Run-Test/Idle
}

// Write the specified 15-bit AVR command
//
uint16 avrWriteCommand(uint16 cmd) {
	uint16 response;
	jtagGotoShiftState();  // Now in Shift-DR
	response = jtagExchangeData16(cmd, 15);  // Now in Exit1-DR
	jtagGotoIdleState();          // Now in Run-Test/Idle
	return response;
}

// Returns a long-word:
//
//   Bits 0-7  : Lock bits
//   Bits 8-15 : Fuse low byte
//   Bits 16-23: Fuse high byte
//   Bits 24-31: Fuse ext. byte
//
uint32 avrReadFuses(void) {
	uint32 result = 0;
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_8A_ENTER_FUSE_READ);
	avrWriteCommand(CMD_8F_READ_FUSES);
	result |= avrWriteCommand(CMD_8F_READ_EXT_BYTE) & 0x00FF;
	result <<= 8;
	result |= avrWriteCommand(CMD_8F_READ_HIGH_BYTE) & 0x00FF;
	result <<= 8;
	result |= avrWriteCommand(CMD_8F_READ_LOW_BYTE) & 0x00FF;
	result <<= 8;
	result |= avrWriteCommand(CMD_8F_READ_LOCK_BITS) & 0x00FF;
	return result;
}

// Accepts a long-word:
//
//   Bits 0-7  : Lock bits (TODO: implement lock bits)
//   Bits 8-15 : Fuse low byte
//   Bits 16-23: Fuse high byte
//   Bits 24-31: Fuse ext. byte
//
void avrWriteFuses(uint32 fuses) {
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_6A_ENTER_FUSE_WRITE);

	avrWriteCommand(CMD_LOAD_DATA_LOW_BYTE | ((fuses>>24)&0xFF));
	avrWriteCommand(CMD_6C_WRITE_EXT_BYTE);
	avrWriteCommand(CMD_6C_WRITE_EXT_BYTE & 0xFDFF);
	avrWriteCommand(CMD_6C_WRITE_EXT_BYTE);
	avrWriteCommand(CMD_6C_WRITE_EXT_BYTE);
	while ( !(avrWriteCommand(CMD_6D_POLL_EXT_BYTE) & 0x0200) );

	avrWriteCommand(CMD_LOAD_DATA_LOW_BYTE | ((fuses>>16)&0x9F));  // Disallow JTAG&SPI disabling
	avrWriteCommand(CMD_6F_WRITE_HIGH_BYTE);
	avrWriteCommand(CMD_6F_WRITE_HIGH_BYTE & 0xFDFF);
	avrWriteCommand(CMD_6F_WRITE_HIGH_BYTE);
	avrWriteCommand(CMD_6F_WRITE_HIGH_BYTE);
	while ( !(avrWriteCommand(CMD_6G_POLL_HIGH_BYTE) & 0x0200) );

	avrWriteCommand(CMD_LOAD_DATA_LOW_BYTE | ((fuses>>8)&0xFF));
	avrWriteCommand(CMD_6I_WRITE_LOW_BYTE);
	avrWriteCommand(CMD_6I_WRITE_LOW_BYTE & 0xFDFF);
	avrWriteCommand(CMD_6I_WRITE_LOW_BYTE);
	avrWriteCommand(CMD_6I_WRITE_LOW_BYTE);
	while ( !(avrWriteCommand(CMD_6J_POLL_LOW_BYTE) & 0x0200) );

	avrWriteCommand(CMD_7A_ENTER_LOCK_WRITE);

	avrWriteCommand(CMD_LOAD_DATA_LOW_BYTE | (fuses&0xFF));
	avrWriteCommand(CMD_7C_WRITE_LOCK_BYTE);
	avrWriteCommand(CMD_7C_WRITE_LOCK_BYTE & 0xFDFF);
	avrWriteCommand(CMD_7C_WRITE_LOCK_BYTE);
	avrWriteCommand(CMD_7C_WRITE_LOCK_BYTE);
	while ( !(avrWriteCommand(CMD_7D_POLL_LOCK_BYTE) & 0x0200) );
}

// Begin reading the specified 128-byte page
//
void avrReadFlashBegin(uint16 page) {
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_3A_ENTER_FLASH_READ);
	avrWriteCommand(CMD_LOAD_ADDRESS_HIGH_BYTE | ((page&0x7F)>>2));
	avrWriteCommand(CMD_LOAD_ADDRESS_LOW_BYTE | ((page&0x03)<<6));
	jtagWriteInstruction(INS_PROG_PAGEREAD);

	// Throw away the first eight bits
	jtagGotoShiftState();  // Now in Shift-DR
	jtagExchangeData(0x00);
}

// Begin writing the specified 128-byte page
//
void avrWriteFlashBegin(uint16 page) {
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_2A_ENTER_FLASH_WRITE);
	avrWriteCommand(CMD_LOAD_ADDRESS_HIGH_BYTE | ((page&0x7F)>>2));
	avrWriteCommand(CMD_LOAD_ADDRESS_LOW_BYTE | ((page&0x03)<<6));
	jtagWriteInstruction(INS_PROG_PAGELOAD);
	jtagGotoShiftState();  // Now in Shift-DR...ready to accept 128 bytes
}

void avrWriteFlashEnd(void) {
	jtagGotoIdleState();          // Now in Run-Test/Idle
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_2G_WRITE_FLASH_PAGE);
	avrWriteCommand(CMD_2G_WRITE_FLASH_PAGE & 0xFDFF);
	avrWriteCommand(CMD_2G_WRITE_FLASH_PAGE);
	avrWriteCommand(CMD_2G_WRITE_FLASH_PAGE);
	while ( !(avrWriteCommand(CMD_2H_POLL_FLASH_PAGE) & 0x0200) );
}

// Erase the device entirely
//
void avrChipErase(void) {
	jtagWriteInstruction(INS_PROG_COMMANDS);
	avrWriteCommand(CMD_1A_CHIP_ERASE_1);
	avrWriteCommand(CMD_1A_CHIP_ERASE_2);
	avrWriteCommand(CMD_1A_CHIP_ERASE_3);
	avrWriteCommand(CMD_1A_CHIP_ERASE_3);
	while ( !(avrWriteCommand(CMD_1A_POLL_ERASE) & 0x0200) );
}

ParseStatus gotXCOMPLETE(void) {
	return PARSE_SUCCESS;
}

ParseStatus gotXTDOMASK(uint16 length, const uint8 *mask) {
	return PARSE_SUCCESS;
}

ParseStatus gotXSIR(uint8 length, const uint8 *sir) {
	#ifdef DEBUG
		usartSendFlashString(PSTR("gotXSIR("));
		usartSendByteHex(length);
		usartSendFlashString(PSTR(", "));
		usartSendByteHex(*sir);
		usartSendFlashString(PSTR(")\n"));
	#endif
	sir += bitsToBytes(length) - 1;
	// Assume Run-Test/Idle on entry
	jtagClock(TMS);                // Now in Select-DR Scan
	jtagGotoShiftState();          // Now in Shift-IR
	while ( length > 8 ) {
		jtagExchangeData(*sir);      // Stay in Shift-IR
		length -= 8;
		sir--;
	}
	jtagExchangeData8(*sir, length); // Now in Exit1-DR
	jtagGotoIdleState();           // Now in Run-Test/Idle
	return PARSE_SUCCESS;
}

ParseStatus gotXRUNTEST(uint32 value) {
	m_idleCycles = value;
	return PARSE_SUCCESS;
}

ParseStatus gotXREPEAT(uint8 value) {
	#ifndef RETRIES
		m_repeats = value;
	#endif
	return PARSE_SUCCESS;
}

ParseStatus gotXSDRSIZE(uint16 value) {
	return PARSE_SUCCESS;
}

inline void delay(uint32 us) {
	while ( us-- ) {
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}
}

ParseStatus gotXSDRTDO(const uint16 length, const uint8 *const data, const uint8 *const mask) {
	const uint16 offset = bitsToBytes(length);
	const uint8 *dataPtr;
	const uint8 *maskPtr;
	uint16 bitCount;
	uint8 errorOccurred;
	uint8 retryCount = 
	#ifdef RETRIES
		RETRIES;
	#else
		m_repeats;
	#endif
	uint8 byte;
	#ifdef DEBUG
		uint16 i;
		usartSendFlashString(PSTR("gotXSDRTDO("));
		usartSendWordHex(length);
		usartSendFlashString(PSTR(", "));
		for ( i = 0; i < offset; i++ ) {
			usartSendByteHex(data[i]);
		}
		usartSendFlashString(PSTR(", "));
		for ( i = 0; i < offset; i++ ) {
			usartSendByteHex(data[offset+i]);
		}
		usartSendFlashString(PSTR(", mask="));
		for ( i = 0; i < offset; i++ ) {
			usartSendByteHex(mask[i]);
		}
		usartSendFlashString(PSTR(")\n"));
	#endif
	for ( ; ; ) {
		#if defined(DEBUG) && DEBUG > 1
			usartSendFlashString(PSTR("  attempt="));
			#ifdef RETRIES
				usartSendByteHex(RETRIES-retryCount);
			#else
				usartSendByteHex(m_repeats-retryCount);
			#endif
			usartSendByte('\n');
		#endif
		errorOccurred = 0;
		dataPtr = data + offset - 1;
		maskPtr = mask + offset - 1;
		bitCount = length;
		// Assume Run-Test/Idle on entry
		delay(m_idleCycles);
		jtagGotoShiftState();  // Now in Shift-DR
		while ( bitCount > 8 ) {
			byte = jtagExchangeData(*dataPtr);      // Stay in Shift-DR
			#if defined(DEBUG) && DEBUG > 1
				usartSendFlashString(PSTR("    sent="));
				usartSendByteHex(*dataPtr);
				usartSendFlashString(PSTR(" (bitCount=08), received="));
				usartSendByteHex(byte);
				usartSendFlashString(PSTR(", expected="));
				usartSendByteHex(dataPtr[offset]);
				usartSendFlashString(PSTR(", mask="));
				usartSendByteHex(*maskPtr);
				usartSendFlashString(PSTR("\n"));
			#endif
			if ( (byte & *maskPtr) != dataPtr[offset] ) {
				errorOccurred = 1;
			}
			bitCount -= 8;
			dataPtr--;
			maskPtr--;
		}
		byte = jtagExchangeData8(*dataPtr, bitCount); // Now in Exit1-DR
		#if defined(DEBUG) && DEBUG > 1
			usartSendFlashString(PSTR("    sent="));
			usartSendByteHex(*dataPtr);
			usartSendFlashString(PSTR(" (bitCount="));
			usartSendByteHex(bitCount);
			usartSendFlashString(PSTR("), received="));
			usartSendByteHex(byte);
			usartSendFlashString(PSTR(", expected="));
			usartSendByteHex(dataPtr[offset]);
			usartSendFlashString(PSTR(", mask="));
			usartSendByteHex(*maskPtr);
			usartSendFlashString(PSTR("\n"));
		#endif
		if ( (byte & *maskPtr) != dataPtr[offset] ) {
			errorOccurred = 1;
		}
		if ( errorOccurred ) {
			if ( --retryCount ) {
				jtagClock(0);    // Now in Pause-DR
				jtagClock(TMS);  // Now in Exit2-DR
				jtagClock(0);    // Now in Shift-DR
				jtagClock(TMS);  // Now in Exit1-DR
				jtagClock(TMS);  // Now in Update-DR
				jtagClock(0);    // Now in Run-Test/Idle
				// ...and try again
			} else {
				// reached maxRetries, give up
				jtagGotoIdleState();  // Now in Run-Test/Idle
				#if defined(DEBUG) && DEBUG > 1
					usartSendFlashString(PSTR("  failed!\n"));
				#endif
				m_failures++;
				return PARSE_SUCCESS;
			}
		} else {
			jtagGotoIdleState();  // Now in Run-Test/Idle
			#if defined(DEBUG) && DEBUG > 1
				usartSendFlashString(PSTR("  success!\n"));
			#endif
			return PARSE_SUCCESS;
		}
	}
}

ParseStatus gotXSTATE(TAPState value) {
	/*jtagReset();
	switch ( state ) {
		case TAPSTATE_TEST_LOGIC_RESET:
			break;
		case TAPSTATE_RUN_TEST_IDLE:
			jtagClock(0);        // Now in Run-Test/Idle
			break;
		case TAPSTATE_SELECT_IR:
	*/
	return PARSE_SUCCESS;
}

void EVENT_USB_Device_UnhandledControlRequest(void) {
	switch ( USB_ControlRequest.bRequest ) {
		case CMD_RD_IDCODE:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
				// Read IDCODE, status, failure count
				uint32 response[3];
				DDRB = TCK | TMS | TDI;
				response[0] = jtagResetAndGetIdentRegister();
				response[1] = m_status;
				response[2] = m_failures;
				PORTB = 0x00;
				DDRB = 0x00;
				Endpoint_ClearSETUP();
				Endpoint_Write_Control_Stream_LE(response, 12);
				Endpoint_ClearStatusStage();
			}
			break;
		case CMD_RW_AVR_FUSES:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
				// Read AVR fuses
				uint32 response;
				DDRB = TCK | TMS | TDI;
				jtagReset();
				avrResetEnable(1);
				avrProgModeEnable(1);
				response = avrReadFuses();
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
				Endpoint_ClearSETUP();
				Endpoint_Write_Control_Stream_LE(&response, 4);
				Endpoint_ClearStatusStage();
			} else if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				// Write AVR fuses
				usartSendFlashString(PSTR("Setting fuses to "));
				usartSendLongHex(((uint32)USB_ControlRequest.wValue << 16) + USB_ControlRequest.wIndex);
				usartSendByte('\n');
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);
				avrWriteFuses(((uint32)USB_ControlRequest.wValue << 16) + USB_ControlRequest.wIndex);
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
			}
			break;
		case CMD_RD_AVR_FLASH:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				// Read AVR flash
				uint8 response[CHUNK_SIZE];
				uint8 i;
				uint16 page;
				uint32 count;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);

				page = 0;
				count = USB_ControlRequest.wValue;
				count <<= 16;
				count += USB_ControlRequest.wIndex;
				count >>= 7;  // number of 128-byte pages
				Endpoint_SelectEndpoint(IN_ENDPOINT_ADDR);
				while ( count-- ) {
					avrReadFlashBegin(page++);
					for ( i = 0; i < CHUNK_SIZE; i++ ) {
						response[i] = jtagExchangeData(0x00);
					}
					Endpoint_Write_Stream_LE(response, CHUNK_SIZE);
					for ( i = 0; i < 63; i++ ) {
						response[i] = jtagExchangeData(0x00);
					}
					response[63] = jtagExchangeDataEnd(0x00);
					jtagGotoIdleState();
					Endpoint_Write_Stream_LE(response, CHUNK_SIZE);
				}
				Endpoint_ClearIN();
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
			}
			break;
		case CMD_WR_AVR_FLASH:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				// Write AVR flash
				uint8 buffer[CHUNK_SIZE];
				uint8 i;
				uint16 page;
				uint32 count;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);

				page = 0;
				count = USB_ControlRequest.wValue;
				count <<= 16;
				count += USB_ControlRequest.wIndex;
				count >>= 7;  // number of 128-byte pages
				Endpoint_SelectEndpoint(OUT_ENDPOINT_ADDR);
				while ( !Endpoint_IsOUTReceived() );
				while ( count-- ) {
					Endpoint_Read_Stream_LE(buffer, CHUNK_SIZE);
					avrWriteFlashBegin(page++);
					for ( i = 0; i < CHUNK_SIZE; i++ ) {
						jtagExchangeData(buffer[i]);
					}
					Endpoint_Read_Stream_LE(buffer, CHUNK_SIZE);
					for ( i = 0; i < 63; i++ ) {
						jtagExchangeData(buffer[i]);
					}
					jtagExchangeDataEnd(buffer[63]);
					avrWriteFlashEnd();
				}
				Endpoint_ClearOUT();
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
			}
			break;
		case CMD_ERASE_AVR_FLASH:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				// Erase AVR flash
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);
				avrChipErase();
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
			}
			break;
		case CMD_WR_XSVF:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint8 buffer[CHUNK_SIZE];
				uint32 bytesRemaining;
				ParseStatus parseStatus = PARSE_SUCCESS;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				DDRB = TCK | TMS | TDI;
				bytesRemaining = USB_ControlRequest.wValue;
				bytesRemaining <<= 16;
				bytesRemaining |= USB_ControlRequest.wIndex;
				#ifdef DEBUG
					usartSendFlashString(PSTR("total = "));
					usartSendLongHex(bytesRemaining);
					usartSendByte('\n');
				#endif
				m_failures = 0;
				parseInit();
				jtagReset();
				jtagClock(0);        // Now in Run-Test/Idle				
				
				Endpoint_SelectEndpoint(OUT_ENDPOINT_ADDR);
				while ( !Endpoint_IsOUTReceived() );
				while ( bytesRemaining >= CHUNK_SIZE && parseStatus == PARSE_SUCCESS ) {
					Endpoint_Read_Stream_LE(buffer, CHUNK_SIZE);
					parseStatus = parse(buffer, CHUNK_SIZE);
					bytesRemaining -= CHUNK_SIZE;
				}
				if ( parseStatus == PARSE_SUCCESS ) {
					// If all is well, read the last few bytes (if any)...
					if ( bytesRemaining ) {
						Endpoint_Read_Stream_LE(buffer, bytesRemaining);
						parseStatus = parse(buffer, bytesRemaining);
					}
				} else {
					// An error occurred, throw away the remaining bytes...
					while ( bytesRemaining >= CHUNK_SIZE ) {
						Endpoint_Read_Stream_LE(buffer, CHUNK_SIZE);
						bytesRemaining -= CHUNK_SIZE;
					}
					if ( bytesRemaining ) {
						Endpoint_Read_Stream_LE(buffer, bytesRemaining);
					}
				}
				m_status = parseStatus;
				Endpoint_ClearOUT();
				PORTB = 0x00;
				DDRB = 0x00;
			}
			break;
	}
}
