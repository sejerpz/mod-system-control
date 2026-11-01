/*
 * This file is part of mod-system-control.
 */

#pragma once

#include "serial.h"

#define _GNU_SOURCE
#include <stdbool.h>
#include <stdint.h>

typedef enum sp_read_error_status {
    /* there was nothing to read, try again */
    SP_READ_ERROR_NO_DATA = -1,
    /* we read something, but data was invalid. call serial_read_ignore_until_zero next */
    SP_READ_ERROR_INVALID_DATA = -2,
    /* IO error while reading, likely due to serial device being disconnected */
    SP_READ_ERROR_IO = -3,
} sp_read_error_status;

sp_read_error_status serial_read_msg_until_zero(struct sp_port* serialport, char buf[0xff], bool debug);

// returns 0 if ok, otherwise an error
sp_read_error_status serial_read_ignore_until_zero(struct sp_port* serialport);

// Reads one whole message, everything up to its null byte, whatever it is: a command or a response.
// Waits up to `timeout_ms` for it to start, then a short while per byte.
// Returns the message length, or an error; a message too long for `buf` is skipped and reported invalid.
// TODO: serial_read_msg_until_zero() could be this plus the sys_ format checks, but it leaves the rest of an
// invalid message for serial_read_ignore_until_zero(), and test.c checks that; test-fake does not link either.
int serial_read_message(struct sp_port* serialport, char buf[0xff], int timeout_ms);

// returns false on IO error, which will automatically close the serial
bool write_or_close(struct sp_port* serialport, const char* msg);

// NOTE: DO NOT USE, needed only for tests
bool serial_read_response(struct sp_port* serialport, char buf[0xff]);
