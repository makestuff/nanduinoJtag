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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "types.h"
#include "usbwrap.h"
#include "buffer.h"
#include "argtable2.h"
#include "arg_uint.h"
#include "dump.h"
#include "../commands.h"

#ifdef WIN32
typedef char *WriteDataPtr;
#pragma warning(disable : 4996)
#else
typedef const char *WriteDataPtr;
#endif

#define BLOCK_SIZE 128
#define TIMEOUT 5000000

typedef enum {
	ATMEL = 0,
	XILINX
} ManufacturerIndex;
static const char *manufacturers[] = {
	"ATMEL",
	"XILINX"
};

typedef struct {
	ManufacturerIndex Manufacturer;
	const char *DeviceID;
	uint8 IRLen;
	uint16 NumBlocks;
} Device;
typedef enum {
	ATMEGA162 = 0,
	XC9572,
	XC3S200,
	XCF02S
} DeviceIndex;
static Device devices[] = {
	{ATMEL,  "ATMEGA162", 4, 16384/BLOCK_SIZE},
	{XILINX, "XC9572",    8, 0},
	{XILINX, "XC3S200",   6, 0},
	{XILINX, "XCF02S",    8, 0}
};

const Device *getDevice(uint16 manufacturerID, uint16 deviceID) {
	if ( manufacturerID == 0x01F ) {
		// Atmel
		if ( deviceID == 0x9404 ) {
			return &devices[ATMEGA162];
		} else {
			return NULL;
		}
	} else if ( manufacturerID == 0x49 ) {
		// Xilinx
		if ( deviceID == 0x9504 ) {
			return &devices[XC9572];
		} else if ( deviceID == 0x1414 ) {
			return &devices[XC3S200];
		} else if ( deviceID == 0x5045 ) {
			return &devices[XCF02S];
		} else {
			return NULL;
		}
	} else {
		return NULL;
	}
}

int controlMsgRead(UsbDeviceHandle *deviceHandle, CommandByte bRequest,
                   uint16 wValue, uint16 wIndex,
                   uint8 *responseData, uint16 wLength)
{
	int returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		(uint8)bRequest,      // bRequest
		wValue,               // wValue
		wIndex,               // wIndex
		(char *)responseData, // space for received data
		wLength,              // wLength
		TIMEOUT               // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
		return 1;
	}
	return 0;
}

int controlMsgWrite(UsbDeviceHandle *deviceHandle, CommandByte bRequest,
                    uint16 wValue, uint16 wIndex,
                    const uint8 *requestData, uint16 wLength)
{
	int returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		(uint8)bRequest,     // bRequest
		wValue,              // wValue
		wIndex,              // wIndex
		(char *)requestData, // space for received data
		wLength,             // wLength
		TIMEOUT              // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
		return 1;
	}
	return 0;
}

int bulkWrite(UsbDeviceHandle *deviceHandle, CommandByte bRequest, const Buffer *buf) {
	int returnCode;
	if ( controlMsgWrite(deviceHandle, bRequest, buf->length >> 16, buf->length & 0xFFFF, NULL, 0x0000) ) {
		return 1;
	}
	returnCode = usb_bulk_write(
		deviceHandle,
		USB_ENDPOINT_OUT | 2,    // write to endpoint 2
		(WriteDataPtr)buf->data, // write from this buffer
		buf->length,             // write entire buffer
		TIMEOUT                  // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		fprintf(stderr, "usb_bulk_write() failed returnCode %d: %s\n", returnCode, usb_strerror());
		return 2;
	}
	return 0;
}

int bulkRead(UsbDeviceHandle *deviceHandle, CommandByte bRequest, Buffer *buf, uint32 length) {
	int returnCode;
	bufZeroLength(buf);
	if ( bufAppendConst(buf, length, 0xFF, NULL) ) {
		fprintf(stderr, "%s\n", bufStrError());
		return 1;
	}
	if ( controlMsgWrite(deviceHandle, bRequest, length >> 16, length & 0xFFFF, NULL, 0x0000) ) {
		return 2;
	}
	returnCode = usb_bulk_read(
		deviceHandle,
		USB_ENDPOINT_IN | 1,  // read from endpoint 1
		(char *)buf->data,    // read into this buffer
		length,               // read "length" bytes
		TIMEOUT               // timeout in milliseconds
	);
	if ( returnCode < 0 ) {
		fprintf(stderr, "usb_bulk_write() failed returnCode %d: %s\n", returnCode, usb_strerror());
		return 3;
	}
	return 0;
}

int main(int argc, char **argv) {
	struct arg_uint *devIndex = arg_uint0("d", "device", "<num>", "    target device");
	struct arg_lit *erase = arg_lit0("e",   "erase",       "           erase the flash, lock bits & maybe EEPROM");
	struct arg_uint *fuses = arg_uint0("f", "fuses",   "<fuses>",  "   set fuses (EX:HI:LO:LK)");
	struct arg_file *load = arg_file0("i",  "load",    "<inFile>", "   load flash from file");
	struct arg_file *save = arg_file0("o",  "save",    "<outFile>", "  save flash to file");
	struct arg_lit *help  = arg_lit0("h",   "help",        "            print this help and exit");
	struct arg_end *end   = arg_end(20);
	void* argTable[] = {devIndex, erase, fuses, load, save, help, end};
	const char *progName = "nj";
	uint32 exitCode = 0;
	int numErrors;
	int returnCode;
	union {
		uint8 bytes[16*sizeof(uint32)];
		uint32 ints[16];
	} u;
	uint32 ident;
	uint16 deviceID, manufacturerID;
	uint8 revision;
	const Device* devices[16];
	const Device *device = NULL;
	uint8 numDevices, firstUnrecognised, i;
	UsbDeviceHandle *deviceHandle;
	Buffer buf;

	printf("NanduinoJTAG Copyright (C) 2010 Chris McClelland\n");

	if ( arg_nullcheck(argTable) != 0 ) {
		printf("%s: insufficient memory\n", progName);
		exitCode = 1;
		goto cleanupArgtable;
	}

	numErrors = arg_parse(argc, argv, argTable);

	if ( help->count > 0 ) {
		printf("Usage: %s", progName);
		arg_print_syntax(stdout, argTable, "\n");
		printf("\nInteract with NanduinoJTAG.\n\n");
		arg_print_glossary(stdout, argTable,"  %-10s %s\n");
		exitCode = 0;
		goto cleanupArgtable;
	}

	if ( numErrors > 0 ) {
		arg_print_errors(stdout, end, progName);
		printf("Try '%s --help' for more information.\n", progName);
		exitCode = 2;
		goto cleanupArgtable;
	}

	if ( bufInitialise(&buf, 1024, 0xFF) != BUF_SUCCESS ) {
		fprintf(stderr, "Cannot allocate buffer: %s\n", bufStrError());
		exitCode = 3;
		goto cleanupArgtable;
	}

	usbInitialise();
	returnCode = usbOpenDevice(0x03EB, 0x3002, 1, 0, 0, &deviceHandle);
	if ( returnCode ) {
		fprintf(stderr, "usbOpenDevice() failed returnCode %d: %s\n", returnCode, usbStrError());
		exitCode = 4;
		goto cleanupBuffer;
	}

	//usb_clear_halt(deviceHandle, 2);

	if ( controlMsgRead(deviceHandle, CMD_SCAN, 0, 0, u.bytes, 64) ) {
		exitCode = 5;
		goto cleanupUsb;
	}
	i = 0;
	while ( i < 16 && u.ints[i] ) {
		i++;
	}
	numDevices = i;
	if ( numDevices == 0 ) {
		fprintf(stderr, "No devices found in the JTAG chain!\n");
		exitCode = 6;
		goto cleanupUsb;
	} else if ( numDevices == 1 ) {
		printf("Found one device in the JTAG chain:\n");
	} else {
		printf("Found %d devices in the JTAG chain:\n", numDevices);
	}
	firstUnrecognised = numDevices;
	for ( i = 0; i < numDevices; i++ ) {
		ident = u.ints[numDevices - 1 - i];
		revision = (ident >> 28) + 'A';
		deviceID = (ident >> 12) & 0xFFFF;
		manufacturerID = (ident >> 1) & 0x07FF;
		devices[i] = getDevice(manufacturerID, deviceID);
		if ( !devices[i] ) {
			if ( firstUnrecognised == numDevices ) {
				firstUnrecognised = i;
			}
			printf("  Device %d (IDCODE=0x%08lX): Unrecognised device 0x%04X/0x%04X\n", i, ident, manufacturerID, deviceID);
		} else {
			printf("  Device %d (IDCODE=0x%08lX): %s %s (rev %c)\n", i, ident, manufacturers[devices[i]->Manufacturer], devices[i]->DeviceID, revision);
		}
	}
	for ( i = 0; i < numDevices; i++ ) {
		if ( !devices[i] ) {
			u.bytes[i] = 0xFF;  // Assume very long irLen
		} else {
			u.bytes[i] = devices[i]->IRLen;
		}
	}

	if ( controlMsgWrite(deviceHandle, CMD_SET_IRLENS, numDevices, 0, u.bytes, numDevices) ) {
		fprintf(stderr, "Call to CMD_SET_IRLENS failed; this should not happen!\n");
		exitCode = 7;
		goto cleanupUsb;
	}

	if ( devIndex->count && devIndex->ival[0] != 0 ) {
		fprintf(stderr, "This version of %s can only directly address the first device in the JTAG chain\n", progName);
		exitCode = 8;
		goto cleanupUsb;
	}

	if ( devIndex->count ) {
		if ( devIndex->ival[0] >= numDevices ) {
			fprintf(stderr, "There is no device numbered %d!\n", devIndex->ival[0]);
			exitCode = 9;
			goto cleanupUsb;
		}
		if ( devIndex->ival[0] >= firstUnrecognised ) {
			fprintf(stderr, "Device %d is either itself unrecognised or is preceded by an unrecognised device.\n", devIndex->ival[0]);
			exitCode = 10;
			goto cleanupUsb;
		}
		device = devices[devIndex->ival[0]];
	} else {
		device = NULL;
		if ( fuses->count || erase->count ) {
			fprintf(stderr, "You must select the target device!\n");
			exitCode = 11;
			goto cleanupUsb;
		}
	}

	if ( device && device->Manufacturer == ATMEL ) {
		if ( controlMsgRead(deviceHandle, CMD_RW_AVR_FUSES, 0, 0, u.bytes, 4) ) {
			exitCode = 12;
			goto cleanupUsb;
		}
		printf("Fuses = 0x%08lX (EX:HI:LO:LK)\n", u.ints[0]);
	}

	if ( fuses->count ) {
		if ( device->Manufacturer == ATMEL ) {
			printf("Setting fuses to 0x%08X\n", fuses->ival[0]);
			if ( controlMsgWrite(deviceHandle, CMD_RW_AVR_FUSES,
								 fuses->ival[0] >> 16,     // wValue: extByte<<8 | highByte
								 fuses->ival[0] & 0xFFFF,  // wIndex: lowByte<<8 | lockBits
								 NULL, 0) )
			{
				exitCode = 13;
				goto cleanupUsb;
			}
		} else {
			fprintf(stderr, "Setting fuses is only supported on Atmel devices\n");
			goto cleanupUsb;
		}
	}

	if ( erase->count ) {
		if ( device->Manufacturer == ATMEL ) {
			printf("Erasing chip...\n");
			if ( controlMsgWrite(deviceHandle, CMD_ERASE_AVR_FLASH, 0, 0, NULL, 0) ) {
				exitCode = 14;
				goto cleanupUsb;
			}
		} else {
			fprintf(stderr, "Erasing is only supported on Atmel devices\n");
			exitCode = 15;
			goto cleanupUsb;
		}
	}

	if ( load->count ) {
		const char *fileName = load->filename[0];
		if ( !strcmp(fileName + strlen(fileName) - 5, ".xsvf") ) {
			printf("Playing XSVF file %s...\n", fileName);
			if ( bufAppendFromBinaryFile(&buf, fileName) ) {
				fprintf(stderr, "Cannot load: %s\n", bufStrError());
				exitCode = 16;
				goto cleanupUsb;
			}
			if ( bulkWrite(deviceHandle, CMD_PLAY_XSVF, &buf) ) {
				exitCode = 17;
				goto cleanupUsb;
			}
		} else if ( !strcmp(fileName + strlen(fileName) - 4, ".hex") ) {
			if ( device ) {
				if ( device->Manufacturer == ATMEL ) {
					uint32 extraBytes, numBlocks;
					printf("Programming Atmel chip using HEX file %s...\n", fileName);
					if ( bufReadFromIntelHexFile(&buf, NULL, fileName) ) {
						fprintf(stderr, "Cannot load: %s\n", bufStrError());
						exitCode = 18;
						goto cleanupBuffer;
					}
					numBlocks = (buf.length % BLOCK_SIZE) ?
						(buf.length / BLOCK_SIZE) + 1 :
						(buf.length / BLOCK_SIZE);
					if ( numBlocks > device->NumBlocks ) {
						fprintf(
							stderr,
							"%s contains 0x%08lX bytes which is too big for the %s which only has 0x%08X bytes of flash",
							fileName,
							buf.length,
							device->DeviceID,
							BLOCK_SIZE * device->NumBlocks
						);
						exitCode = 19;
						goto cleanupUsb;
					}
					extraBytes = BLOCK_SIZE * numBlocks - buf.length;
					if ( bufAppendConst(&buf, extraBytes, 0xFF, NULL) ) {
						fprintf(stderr, "%s\n", bufStrError());
						exitCode = 20;
						goto cleanupUsb;
					}
					if ( bulkWrite(deviceHandle, CMD_WR_AVR_FLASH, &buf) ) {
						exitCode = 21;
						goto cleanupUsb;
					}
				} else {
					fprintf(stderr, "Loading HEX files is only supported on Atmel devices\n");
					exitCode = 22;
					goto cleanupUsb;
				}
			} else {
				fprintf(stderr, "You must select the target device!\n");
				exitCode = 23;
				goto cleanupUsb;
			}
		} else {
			fprintf(stderr, "File %s has unrecognised extension\n", fileName);
			exitCode = 24;
			goto cleanupUsb;
		}
		if ( controlMsgRead(deviceHandle, CMD_STATUS, 0, 0, u.bytes, 8) ) {
			exitCode = 25;
			goto cleanupUsb;
		}
		printf("Load operation completed with returncode 0x%08lX, numfails=%lu\n", u.ints[0], u.ints[1]);
	}

	if ( save->count ) {
		const char *fileName = save->filename[0];
		if ( !strcmp(fileName + strlen(fileName) - 4, ".hex") ) {
			if ( device ) {
				if ( device->Manufacturer == ATMEL ) {
					if ( bulkRead(deviceHandle, CMD_RD_AVR_FLASH, &buf, BLOCK_SIZE * device->NumBlocks) ) {
						exitCode = 26;
						goto cleanupUsb;
					}
					if ( bufWriteToIntelHexFile(&buf, NULL, fileName, 16, true) ) {
						fprintf(stderr, "Cannot write hex records: %s\n", bufStrError());
						exitCode = 27;
						goto cleanupUsb;
					}
				} else {
					fprintf(stderr, "Saving HEX files is only supported on Atmel devices\n");
					exitCode = 28;
					goto cleanupUsb;
				}
			} else {
				fprintf(stderr, "You must select the target device!\n");
				exitCode = 29;
				goto cleanupUsb;
			}
		} else {
			fprintf(stderr, "File %s has unrecognised extension\n", fileName);
			exitCode = 30;
			goto cleanupUsb;
		}
		if ( controlMsgRead(deviceHandle, CMD_STATUS, 0, 0, u.bytes, 8) ) {
			exitCode = 31;
			goto cleanupUsb;
		}
		printf("Save operation completed with returncode 0x%08lX, numfails=%lu\n", u.ints[0], u.ints[1]);
	}

	cleanupUsb:
		usb_release_interface(deviceHandle, 0);
		usb_close(deviceHandle);

	cleanupBuffer:
		bufDestroy(&buf);

	cleanupArgtable:
		arg_freetable(argTable, sizeof(argTable)/sizeof(argTable[0]));

	//getchar();
	return exitCode;
}
