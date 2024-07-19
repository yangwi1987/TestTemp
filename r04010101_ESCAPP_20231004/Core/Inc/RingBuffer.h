/*
 * RingBuffer.h
 *
 *  Created on: 2024年7月19日
 *      Author: User
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include <stdlib.h>
#define BUFFER_SIZE 5000 // Define the size of the ring buffer

typedef struct {
    float *buffer;
    int head;
    int tail;
    int max;
    int count;
} RingBuffer;

// Function prototypes
void initBuffer(RingBuffer *rb, int size);
void freeBuffer(RingBuffer *rb);
int isFull(RingBuffer *rb);
int isEmpty(RingBuffer *rb);
void addToBuffer(RingBuffer *rb, float data);
int getFromBuffer(RingBuffer *rb);
//void recordVariableChange(RingBuffer *rb, float *variable, float newValue);


#endif /* INC_RINGBUFFER_H_ */
