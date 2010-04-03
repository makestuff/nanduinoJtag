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

static uint32 magic = 0;

int main(void) {
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	PORTB = 0x00;
	DDRB = 0x00;
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
	magic = 0;
	if ( !(Endpoint_ConfigureEndpoint(IN_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_IN,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		magic |= 0xDEAD0000;
	}
	if ( !(Endpoint_ConfigureEndpoint(OUT_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_OUT,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		magic |= 0x0000DEAD;
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
	if ( jtagClock(data&0x01 ? TDI : 0) ) {
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

void EVENT_USB_Device_UnhandledControlRequest(void) {
	switch ( USB_ControlRequest.bRequest ) {
		case 0x80:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
				uint32 response[3];
				DDRB = TCK | TMS | TDI;
				response[0] = jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);
				response[1] = avrReadFuses();
				response[2] = magic;
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
				Endpoint_ClearSETUP();
				Endpoint_Write_Control_Stream_LE(response, 12);
				Endpoint_ClearStatusStage();
			} else if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
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
		case 0x81:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint8 response[64];
				uint8 i;
				uint16 page, count;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);

				page = USB_ControlRequest.wValue;
				count = USB_ControlRequest.wIndex;

				Endpoint_SelectEndpoint(IN_ENDPOINT_ADDR);
				while ( count-- ) {
					avrReadFlashBegin(page++);
					for ( i = 0; i < 64; i++ ) {
						response[i] = jtagExchangeData(0x00);
					}
					Endpoint_Write_Stream_LE(response, 64);
					for ( i = 0; i < 63; i++ ) {
						response[i] = jtagExchangeData(0x00);
					}
					response[63] = jtagExchangeDataEnd(0x00);
					jtagGotoIdleState();
					Endpoint_Write_Stream_LE(response, 64);
				}
				Endpoint_ClearIN();
				avrProgModeEnable(0);
				avrResetEnable(0);
				PORTB = 0x00;
				DDRB = 0x00;
			}
			break;
		case 0x82:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
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
		case 0x83:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint8 buffer[64];
				uint8 i;
				uint16 page, count;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
				DDRB = TCK | TMS | TDI;
				jtagResetAndGetIdentRegister();
				avrResetEnable(1);
				avrProgModeEnable(1);

				page = USB_ControlRequest.wValue;
				count = USB_ControlRequest.wIndex;

				Endpoint_SelectEndpoint(OUT_ENDPOINT_ADDR);
				while ( !Endpoint_IsOUTReceived() );
				while ( count-- ) {
					Endpoint_Read_Stream_LE(buffer, 64);
					avrWriteFlashBegin(page++);
					for ( i = 0; i < 64; i++ ) {
						jtagExchangeData(buffer[i]);
					}
					Endpoint_Read_Stream_LE(buffer, 64);
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
	}
}
