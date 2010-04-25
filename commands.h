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
#ifndef COMMANDS_H
#define COMMANDS_H

typedef enum {
	CMD_SCAN = 0x80,
	CMD_RW_AVR_FUSES,
	CMD_RD_AVR_FLASH,
	CMD_WR_AVR_FLASH,
	CMD_ERASE_AVR_FLASH,
	CMD_RSVD1,
	CMD_RSVD2,
	CMD_RSVD3,
	CMD_PLAY_XSVF,
	CMD_STATUS
} CommandByte;

#endif
