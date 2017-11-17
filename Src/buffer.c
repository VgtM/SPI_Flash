/*
 * buffer.c
 *
 *  Created on: Sep 26, 2017
 *      Author: spinkoh
 */

#include "buffer.h"
#include <string.h>
#include <stdlib.h>
#include <time.h>

void init_buffer(triple_ring_buffer* buffer)
{
	//Initialize the arrays to 0

	memset(buffer->tripletBuffer, 0 ,sizeof(float) * 3 * SIZE);

	buffer->head = 0;
	buffer->tail = 1;
	return;
}

int add(triple_ring_buffer* buffer,triplet dat)
{
	if(isFull(buffer) == BUFFER_AVAILABLE)
	{
		(buffer->tripletBuffer[buffer->tail]).x = dat.x;
		(buffer->tripletBuffer[buffer->tail]).y = dat.y;
		(buffer->tripletBuffer[buffer->tail]).z = dat.z;

		buffer->tail = (buffer->tail + 1) % SIZE;
		return BUFFER_SUCCESS;
	}

	return BUFFER_FAILED;
}

int fetch(triple_ring_buffer* buffer, triplet* xyzTriplet)
{
	if(peek(buffer) == BUFFER_AVAILABLE)
	{
		xyzTriplet->x = (buffer->tripletBuffer[buffer->head]).x;
		xyzTriplet->y = (buffer->tripletBuffer[buffer->head]).y;
		xyzTriplet->z = (buffer->tripletBuffer[buffer->head]).z;

		buffer->head = (buffer->head + 1) % SIZE;

		return BUFFER_SUCCESS;
	}

	return BUFFER_FAILED;
}
int fetch_previous(triple_ring_buffer* buffer, triplet* xyzTriplet)
{
	if(peek(buffer) == BUFFER_AVAILABLE)
	{
		xyzTriplet->x = (buffer->tripletBuffer[(buffer->head -1) % SIZE]).x;
		xyzTriplet->y = (buffer->tripletBuffer[(buffer->head -1) % SIZE]).y;
		xyzTriplet->z = (buffer->tripletBuffer[(buffer->head -1) % SIZE]).z;

		return BUFFER_SUCCESS;
	}

	return BUFFER_FAILED;
}

int peek(triple_ring_buffer* buffer)
{
	if((buffer->head + 1) % SIZE != buffer->tail) return BUFFER_AVAILABLE;
	else return BUFFER_FAILED;
}

int isFull(triple_ring_buffer* buffer)
{
	if((buffer->tail + 1) % SIZE == buffer->head) return BUFFER_FULL;
	else return BUFFER_AVAILABLE;
}

int fetchBuffer(triple_ring_buffer* buffer1, float* buffer2, float* buffer3, float* buffer4){
	if(peek(buffer1) == BUFFER_AVAILABLE)
	{
		*buffer2 = (buffer1->tripletBuffer[buffer1->head]).x;
		*buffer3 = (buffer1->tripletBuffer[buffer1->head]).y;
		*buffer4 = (buffer1->tripletBuffer[buffer1->head]).z;
		buffer1->head = (buffer1->head + 1) % SIZE;

		return BUFFER_SUCCESS;
	}

	return BUFFER_FAILED;
}

int randGyroReading(triplet* buffer){
	buffer->x = 0;
	buffer->y = 0;
	buffer->z = 0;

	srand(time(NULL));
	buffer->x = (float)rand()/(float)(RAND_MAX/5);
	buffer->y = (float)rand()/(float)(RAND_MAX/5);
	buffer->z = (float)rand()/(float)(RAND_MAX/5);
	return 0;
}
