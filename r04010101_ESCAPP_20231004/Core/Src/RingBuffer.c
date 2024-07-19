/*
 * RingBuffer.c
 *
 *  Created on: 2024年7月19日
 *      Author: User
 */


#include <RingBuffer.h>


// Initialize the ring buffer
void initBuffer(RingBuffer *rb, int size) {
    rb->buffer = (float *)malloc(size * sizeof(float));
    rb->max = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

// Free the ring buffer
void freeBuffer(RingBuffer *rb) {
    free(rb->buffer);
}

// Check if the buffer is full
int isFull(RingBuffer *rb) {
    return rb->count == rb->max;
}

// Check if the buffer is empty
int isEmpty(RingBuffer *rb) {
    return rb->count == 0;
}

// Add data to the ring buffer
void addToBuffer(RingBuffer *rb, float data) {
	static int flag = 0;
    if (isFull(rb)) {
        rb->tail = (rb->tail + 1) % rb->max;
    } else {
        rb->count++;
    }

    if(rb->head == (rb->max - 1))
    {
    	flag = 1;
    }
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->max;
}

// Get data from the ring buffer
int getFromBuffer(RingBuffer *rb) {
    if (isEmpty(rb)) {
        return -1; // Return -1 if the buffer is empty
    } else {
    	float data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->max;
        rb->count--;
        return data;
    }
}
