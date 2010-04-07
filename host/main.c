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
#include "usbwrap.h"
#include "hexreader.h"
#include "buffer.h"
#include "argtable2.h"
#include "arg_uint.h"
#include "dump.h"

#ifdef WIN32
typedef char *WriteDataPtr;
#pragma warning(disable : 4996)
#else
typedef const char *WriteDataPtr;
#endif

typedef struct {
	const char *Manufacturer;
	const char *DeviceID;
	unsigned short NumBlocks;
} Device;

#define ATMEGA162 0
#define XC9572    1
#define BLOCK_SIZE 128

static Device devices[] = {
	{"ATMEL", "ATMEGA162", 16384/BLOCK_SIZE},
	{"XILINX", "XC9572", 0}
};

const Device *getDevice(unsigned short manufacturerID, unsigned short deviceID) {
	if ( manufacturerID == 0x01F ) {
		if ( deviceID == 0x9404 ) {
			return &devices[ATMEGA162];
		} else {
			return NULL;
		}
	} else if ( manufacturerID == 0x49 ) {
		if ( deviceID == 0x9504 ) {
			return &devices[XC9572];
		} else {
			return NULL;
		}
	} else {
		return NULL;
	}
}

#define goexit(n) exitCode = n; goto exit;

int main(int argc, char **argv) {
	struct arg_lit *erase = arg_lit0("e",   "erase",       "           erase the flash, lock bits & maybe EEPROM");
	struct arg_uint *fuses = arg_uint0("f", "fuses",   "<fuses>",  "   set fuses (EX:HI:LO:LK)");
	struct arg_int *base  = arg_int0("b",   "base",    "<baseBlock>", "set base block for flash writes");
	struct arg_file *load = arg_file0("i",  "load",    "<inFile>", "   load flash from file");
	struct arg_file *save = arg_file0("o",  "save",    "<outFile>", "  save flash to file");
	struct arg_lit *debug = arg_lit0("d",   "debug",       "           output debugging information");
	struct arg_lit *help  = arg_lit0("h",   "help",        "            print this help and exit");
	struct arg_end *end   = arg_end(20);
	void* argTable[] = {erase, fuses, base, load, save, debug, help, end};
	const char *progName = "ucm";
	int exitCode = 0;
	int numErrors;
	union {
		unsigned char bytes[3*sizeof(int)];
		unsigned int ints[3];
	} buffer;
	char revision;
	int returnCode;
	unsigned int ident;
	unsigned short deviceID, manufacturerID;
	unsigned long numBlocks = 0;
	const Device *device;
	UsbDeviceHandle *deviceHandle;
	HexReader hexReader;
	HexReaderStatus hStatus;
	Buffer buf;
	BufferStatus bStatus;
	FILE *file;

	if ( arg_nullcheck(argTable) != 0 ) {
		printf("%s: insufficient memory\n", progName);
		goexit(1);
	}

	numErrors = arg_parse(argc, argv, argTable);

	if ( help->count > 0 ) {
		printf("Usage: %s", progName);
		arg_print_syntax(stdout, argTable, "\n");
		printf("\nInteract with NanduinoJTAG.\n\n");
		arg_print_glossary(stdout, argTable,"  %-10s %s\n");
		goexit(0);
	}

	if ( numErrors > 0 ) {
		arg_print_errors(stdout, end, progName);
		printf("Try '%s --help' for more information.\n", progName);
		goexit(1);
	}

	usbInitialise();
	returnCode = usbOpenDevice(0x03EB, 0x3002, 1, 0, 0, &deviceHandle);
	if ( returnCode ) {
		fprintf(stderr, "usbOpenDevice() failed returnCode %d: %s\n", returnCode, usbStrError());
		goexit(1);
	}

	printf("Reading ID & fuses...\n");
	returnCode = usb_control_msg(
		deviceHandle,
		USB_ENDPOINT_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		0x80,    // bRequest
		0x0000,  // wValue
		0x0000,  // wIndex
		(char *)buffer.bytes,  // space for received data
		12,      // wLength
		5000     // timeout after five seconds
	);
	if ( returnCode < 0 ) {
		fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
		goexit(1);
	}
	//dump(0x00000000, buffer.bytes, 12);
	ident = buffer.ints[0];
	revision = (ident >> 28) + 'A';
	deviceID = ident >> 12;
	manufacturerID = (ident >> 1) & 0x07FF;
	device = getDevice(manufacturerID, deviceID);
	if ( !device ) {
		fprintf(stderr, "Unrecognised device: 0x%04X/0x%04X (IDCODE 0x%08X)\n", manufacturerID, deviceID, ident);
		goexit(1);
	}
	printf("Found %s %s (rev %c)\n", device->Manufacturer, device->DeviceID, revision);
	printf("Fuses = 0x%08X (EX:HI:LO:LK)\n", buffer.ints[1]);
	if ( debug->count ) {
		printf("Debug word = 0x%08X\n", buffer.ints[2]);
	}

	if ( fuses->count ) {
		printf("Setting fuses to 0x%08X\n", fuses->ival[0]);
		returnCode = usb_control_msg(
			deviceHandle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,  // bmRequestType
			0x80,    // bRequest
			fuses->ival[0] >> 16,     // wValue: extByte<<8 | highByte
			fuses->ival[0] & 0xFFFF,  // wIndex: lowByte<<8 | lockBits
			NULL,    // data to send
			0x0000,  // send nothing
			5000     // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
	}

	if ( load->count ) {
		int extraBytes;
		printf("Reading %s...\n", load->filename[0]);
		hStatus = hexInitialise(&hexReader, 0xFF);
		if ( hStatus != HEX_SUCCESS ) {
			fprintf(stderr, "%s\n", hexStrError());
			goexit(1);
		}
		file = fopen(load->filename[0], "r");
		if ( !file ) {
			fprintf(stderr, "Cannot open file %s: %s\n", load->filename[0], strerror(errno));
			goexit(1);
		}
		hStatus = hexReadFile(&hexReader, file);
		if ( hStatus != HEX_SUCCESS ) {
			fprintf(stderr, "%s\n", hexStrError());
			goexit(1);
		}
		fclose(file);
		numBlocks = (hexReader.data.length % BLOCK_SIZE) ?
			(hexReader.data.length / BLOCK_SIZE) + 1 :
			(hexReader.data.length / BLOCK_SIZE);
		if ( numBlocks > device->NumBlocks ) {
			fprintf(
				stderr,
				"%s contains 0x%08X bytes which is too big for the %s which only has 0x%08X bytes of flash",
				load->filename[0],
				hexReader.data.length,
				device->DeviceID,
				BLOCK_SIZE * device->NumBlocks
			);
			goexit(1);
		}
		extraBytes = BLOCK_SIZE * numBlocks - hexReader.data.length;
		bStatus = bufAppendConst(&hexReader.data, extraBytes, 0xFF, NULL);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
		bStatus = bufAppendConst(&hexReader.writeMap, extraBytes, 0x00, NULL);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
	}

	if ( erase->count ) {
		printf("Erasing chip...\n");
		returnCode = usb_control_msg(
			deviceHandle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,  // bmRequestType
			0x82,    // bRequest
			0x0000,  // wValue
			0x0000,  // wIndex
			NULL,    // data to send
			0x0000,  // send nothing
			5000     // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
	}

	if ( load->count ) {
		printf("Programming chip...\n");
		returnCode = usb_control_msg(
			deviceHandle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,  // bmRequestType
			0x83,    // bRequest
			base->count ? base->ival[0] : 0x0000,  // wValue: base block
			numBlocks,  // wIndex
			NULL,    // data to send
			0x0000,  // send nothing
			5000     // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
		returnCode = usb_bulk_write(
			deviceHandle,
			USB_ENDPOINT_OUT | 2,   // write to endpoint 2
			(WriteDataPtr)hexReader.data.data, // write from this buffer
			BLOCK_SIZE * numBlocks, // write numBlocks bytes
			5000                    // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_bulk_write() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}

		printf("Validating...\n");
		bStatus = bufInitialise(&buf, 1024, 0);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
		bStatus = bufAppendConst(&buf, BLOCK_SIZE * device->NumBlocks, 0xFF, NULL);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
		returnCode = usb_control_msg(
			deviceHandle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,  // bmRequestType
			0x81,    // bRequest
			0x0000,  // wValue
			device->NumBlocks,  // wIndex
			NULL,    // data to send
			0x0000,  // send nothing
			5000     // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
		returnCode = usb_bulk_read(
			deviceHandle,
			USB_ENDPOINT_IN | 1,   // read from endpoint 1
			(char *)buf.data,  // read into this buffer
			BLOCK_SIZE * device->NumBlocks, // read entire flash
			5000                   // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_bulk_read() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
		if ( memcmp(buf.data, hexReader.data.data, hexReader.data.length) ) {
			printf("ERROR - Validation failed!\n");
		}
		bufDestroy(&buf);
		hexDestroy(&hexReader);
	}

	if ( save->count ) {
		printf("Reading flash...\n");
		hStatus = hexInitialise(&hexReader, 0xFF);
		if ( hStatus != HEX_SUCCESS ) {
			fprintf(stderr, "%s\n", hexStrError());
			goexit(1);
		}
		bStatus = bufAppendConst(&hexReader.data, BLOCK_SIZE * device->NumBlocks, 0xFF, NULL);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
		returnCode = usb_control_msg(
			deviceHandle,
			USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,  // bmRequestType
			0x81,    // bRequest
			0x0000,  // wValue
			device->NumBlocks,  // wIndex
			NULL,    // data to send
			0x0000,  // send nothing
			5000     // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_control_msg() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
		returnCode = usb_bulk_read(
			deviceHandle,
			USB_ENDPOINT_IN | 1,   // read from endpoint 1
			(char *)hexReader.data.data,  // read into this buffer
			BLOCK_SIZE * device->NumBlocks,  // read entire flash
			5000                   // timeout after five seconds
		);
		if ( returnCode < 0 ) {
			fprintf(stderr, "usb_bulk_read() failed returnCode %d: %s\n", returnCode, usb_strerror());
			goexit(1);
		}
		hStatus = hexDeriveWriteMap(&hexReader);
		if ( hStatus != HEX_SUCCESS ) {
			fprintf(stderr, "%s\n", hexStrError());
			goexit(1);
		}
		bStatus = bufInitialise(&buf, 1024, 0);
		if ( bStatus != BUF_SUCCESS ) {
			fprintf(stderr, "%s\n", bufStrError());
			goexit(1);
		}
		hStatus = hexWriteHexRecordsToBuffer(&hexReader, &buf, 16);
		if ( hStatus != HEX_SUCCESS ) {
			fprintf(stderr, "%s\n", hexStrError());
			goexit(1);
		}
		file = fopen(save->filename[0], "w");
		if ( !file ) {
			fprintf(stderr, "Cannot open file %s: %s\n", save->filename[0], strerror(errno));
			goexit(1);
		}
		fprintf(file, "%s", buf.data);
		fclose(file);
		bufDestroy(&buf);
		hexDestroy(&hexReader);
	}
	usb_close(deviceHandle);

exit:
	//getchar();
	arg_freetable(argTable, sizeof(argTable)/sizeof(argTable[0]));

	return exitCode;
}
