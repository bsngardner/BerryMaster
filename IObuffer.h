/*
 * IObuffer.h
 *
 *  Created on: May 20, 2015
 *      Author: Kristian Sims
 */

#ifndef IOBUFFER_H_
#define IOBUFFER_H_

typedef struct
{
	char* buffer;				// pointer to actual buffer in memory
	int size;					// maximum capacity -- size of buffer
	int tail_dex;				// index of oldest data = index to read from
	int count;					// number of bytes in buffer
	void (*bytes_ready)(void);	// callback to wake up writer
} IObuffer;	// 10 bytes...

// TODO: Does this (or its declarations) need to be volatile?

// Support functions

// Writing functions
int IOputc(char, IObuffer*);
int IOputs(const char*, IObuffer*);

// Reading functions
int IOgetc(char*, IObuffer*);

// Auxiliary functions
IObuffer* IObuffer_create(int size);
void IObuffer_destroy(IObuffer* iob);

#endif /* IOBUFFER_H_ */
