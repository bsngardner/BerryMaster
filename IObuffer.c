/*
 * IObuffer.c
 *
 *  Created on: May 20, 2015
 *      Author: Kristian Sims
 */

#include <stdlib.h>
#include "IObuffer.h"

// Write one character to the buffer
int IOputc(char c, IObuffer* iob) {
	int head_dex; // This might be cached for more speed...

	// Return error if buffer is null or inactive or full
	if (!iob || !iob->size || (iob->size - iob->count <= 0))
		return -1; // error
	// TODO: check for null buffer? (Ouch 6/20/2015)

	// Disable interrupts for interrupt-driven IO

	__disable_interrupt();

	// Get pointer for write location and write to it
	head_dex = iob->tail_dex + iob->count;
	if (head_dex >= iob->size)
		head_dex -= iob->size;
	*(iob->buffer + head_dex) = c;

//	*(iob->buffer + (iob->tail_dex+iob->count)%iob->size) = c; // old

	// Signal that there are bytes ready (if it was empty)
	// (check that the callback isn't null. maybe this would be a flag)
	if (!iob->count++ && iob->bytes_ready)
		iob->bytes_ready();

	__enable_interrupt();

	return 0; // success
}

// Write a C string to the buffer (null terminated!)
int IOputs(const char* s, IObuffer* iob) {
	int space_left,		// bytes left to EITHER end of buffer or overflow
			byte_count = 0;		// bytes written
	char* write_ptr;	// pointer to which to copy
	char c = 0xFF;

	// Return error if buffer is null or inactive or full
	if (!iob || !iob->size || (iob->size - iob->count <= 0))
		return -1; // error

	__disable_interrupt();
	// Use space_left as an intermediate variable
	space_left = iob->tail_dex + iob->count;				// index of head
	if (space_left >= iob->size)
		space_left -= iob->size;	// inside of buffer

	// Use the intermediate value to get the pointer to which to write
	write_ptr = iob->buffer + space_left;

	// Calculate space from write head to end of buffer
	space_left = iob->size - space_left;

	// Disable interrupts for interrupt-driven IO

	// If the space from the head to the wrap is empty, write to that
	if (space_left < (iob->size - iob->count)) {
		while (space_left--) {
			c = *s++;	// copy from string
			if (c)
				byte_count++, *write_ptr++ = c;
			else
				break;
		}
	}

	// Reset write_ptr to the beginning if we used the first loop
	if (space_left <= 0)
		write_ptr = iob->buffer;
	space_left = iob->size - iob->count - byte_count;

	// Write until buffer is full (either after wrap or having skipped it)
	if (c) {
		while (space_left--) {
			c = *s++;	// copy from string
			if (c)
				byte_count++, *write_ptr++ = c;
			else
				break;
		}
	}

	// Alert the writer (could be if buffer was empty)
	if (!iob->count && iob->bytes_ready)
		iob->bytes_ready();

	// Increment IObuffer count

	if ((!c || !iob->fit_block))
		iob->count += byte_count;
	else
		__no_operation();

	__enable_interrupt();
	// Could optionally check if more space has been made available (interrupts)

	// Return byte count, or -1 if the full string was not copied
	return c ? -byte_count : byte_count;
	// then maybe check if more space is available (interrupts)
}

//Unfinished, not declared in .h
int IOputnbytes(const char* s, int n, IObuffer* iob) {
	return 0;
}

// Get a character out of the buffer
int IOgetc(char* cp, IObuffer *iob) {
	// Return error if buffer is null or inactive or empty or if cp is null
	if (!iob || !iob->size || !iob->count || !cp)
		return -1; // error

	// Load character where cp points
	*cp = *(iob->buffer + iob->tail_dex);

	// Update tail_dex and count
	iob->tail_dex++;
	if (iob->tail_dex >= iob->size)
		iob->tail_dex -= iob->size;
	iob->count--;

	return 0;
}

IObuffer* IObuffer_create(int size) {
	// Minimum size of 1
	if (size <= 0)
		return 0;

	// Allocate buffer struct
	IObuffer* iob = (IObuffer*) malloc(sizeof(IObuffer));
	if (!iob)
		return 0; // return null if malloc failed

	// Allocate buffer space
	iob->buffer = malloc(size);
	if (!iob->buffer)
		return 0; // return null if malloc failed

	// Initialize struct values
	iob->tail_dex = 0;
	iob->count = 0;
	iob->size = size;
	iob->bytes_ready = 0;

	return iob;
}

void IObuffer_init(IObuffer* iob, char* buffer, int size, void (*cb)(void)) {
	iob->buffer = buffer;
	iob->tail_dex = 0;
	iob->count = 0;
	iob->size = size;
	iob->bytes_ready = cb;
}

void IObuffer_destroy(IObuffer* iob) {
	if (iob) {
		if (iob->buffer)
			free(iob->buffer);
		free(iob);
	}
}
