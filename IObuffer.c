/*
 * IObuffer.c
 *
 *  Created on: May 20, 2015
 *      Author: Kristian Sims
 */
#include "msp430.h"
#include <stdlib.h>
#include "IObuffer.h"

// Write one character to the buffer
int IOputc(char c, IObuffer* iob)
{
	int head_dex; // This might be cached for more speed...

	// Return error if buffer is null or inactive or full
	if (!iob || !iob->size || (iob->size - iob->count <= 0))
		return -1; // error

	// Disable interrupts for interrupt-driven IO

	short sr = __get_SR_register();
	__disable_interrupt();

	// Get pointer for write location and write to it
	head_dex = iob->tail_dex + iob->count;
	if (head_dex >= iob->size)
		head_dex -= iob->size;
	*(iob->buffer + head_dex) = c;

//	*(iob->buffer + (iob->tail_dex+iob->count)%iob->size) = c; // old

	// Signal that there are bytes ready (if it was empty)
	// (check that the callback isn't null. maybe this would be a flag)
	if (!(iob->count++ && iob->callback_once) && iob->bytes_ready)
		iob->bytes_ready();

	__bis_SR_register(sr & GIE);

	return 0; // success
}

// Write a C string to the buffer (null terminated!)
int IOputs(const char* s, IObuffer* iob)
{
	//Counts bytes until null, calls nputs
	const char* ptr = s;
	int n = 0;
	while (*ptr++)
		n++;
	return IOnputs(s, n, iob);
}

//Unfinished, not declared in .h
int IOnputs(const char* src, int n, IObuffer* iob)
{
	int space_left;		// bytes left to EITHER end of buffer or overflow
	char* write_ptr;		// pointer to which to copy

	short sr = __get_SR_register();
	__disable_interrupt();
	// Return error if buffer is null or inactive
	if (!iob || !iob->size)
		return -1; // error
	//Return if buffer is full or bytes will not fit.
	if (n > iob->size - iob->count)
		return -1;

	//Precalulate space left until wrap around
	space_left = iob->tail_dex + iob->count;
	if (space_left >= iob->size)
		space_left -= iob->size;	// inside of buffer
	write_ptr = iob->buffer + space_left; //Sneak in init of write pointer
	space_left = iob->size - space_left;

	int new_count = iob->count + n;

	//If there is more space than n, skip first loop and write
	//	n bytes straight through
	if (space_left < n)
	{
		n -= space_left;
		while (space_left-- > 0)
			*write_ptr++ = *src++;
		write_ptr = iob->buffer;
	}
	while (n-- > 0)
		*write_ptr++ = *src++;

	if (!iob->count && iob->bytes_ready)
		iob->bytes_ready();
	iob->count = new_count;

	__bis_SR_register(sr & GIE);
	return 0;
}

//Unfinished, not declared in .h
int IOnputs_mem(const char* src, int n, IObuffer* iob)
{
	int space_left;		// bytes left to EITHER end of buffer or overflow
	char* write_ptr;		// pointer to which to copy

	// Return error if buffer is null or inactive
	if (!iob || !iob->size)
		return -1; // error
	//Return if buffer is full or bytes will not fit.
	if (n > iob->size - iob->count)
		return -1;

	//Precalulate space left until wrap around
	space_left = iob->tail_dex + iob->count;
	if (space_left >= iob->size)
		space_left -= iob->size;	// inside of buffer
	write_ptr = iob->buffer + space_left; //Sneak in init of write pointer
	space_left = iob->size - space_left;

	iob->count += n;

	//If there is more space than n, skip first loop and write
	//	n bytes straight through
	if (space_left < n)
	{
		memcpy(write_ptr, src, space_left);
		n -= space_left;
		src += space_left;
		write_ptr = iob->buffer;
	}
	memcpy(write_ptr, src, n);

	return 0;
}

// Get a character out of the buffer
int IOgetc(char* cp, IObuffer *iob)
{
	// Return error if buffer is null or inactive or empty or if cp is null
	if (!iob || !iob->size || !iob->count || !cp)
		return -1; // error

	short sr = __get_SR_register();
	__disable_interrupt();

	// Load character where cp points
	*cp = *(iob->buffer + iob->tail_dex);

	// Update tail_dex and count
	iob->tail_dex++;
	if (iob->tail_dex >= iob->size)
		iob->tail_dex -= iob->size;
	iob->count--;

	__bis_SR_register(sr & GIE);

	return 0;
}

IObuffer* IObuffer_create(int size)
{
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
	iob->callback_once = 0;

	return iob;
}

void IObuffer_init(IObuffer* iob, char* buffer, int size, void (*cb)(void))
{
	iob->buffer = buffer;
	iob->tail_dex = 0;
	iob->count = 0;
	iob->size = size;
	iob->bytes_ready = cb;
}

void IObuffer_destroy(IObuffer* iob)
{
	if (iob)
	{
		if (iob->buffer)
		{
			free(iob->buffer);
			iob->buffer = NULL;
		}
		free(iob);
		iob = NULL;
	}
}
