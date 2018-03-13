/*
 * OutputTaskPrivate.h
 *
 *  Created on: Mar 11, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_OUTPUTTASKPRIVATE_H_
#define LEDOUT_OUTPUTTASKPRIVATE_H_

#include <cstddef>

typedef enum {
	/**
	 * Converts the given buffer for output, and calls the completion handler
	 * when the conversion is finished.
	 */
	kOutputMessageConvert					= 1,

	/**
	 * Sends a previously converted buffer to the output.
	 */
	kOutputMessageSend,
} output_message_type_t;

/**
 * Message passed to the output task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// message type
	output_message_type_t type;

	// what channel does this data correspond to?
	unsigned int channel;

	// message-specific payload
	union {
		// buffer conversion
		struct {
			// address and length of buffer, in number of LEDs
			void *buffer;

			unsigned int numLEDs	: 24;
			unsigned int isRGBW	: 1;
		} convert;

		// sending of converted data
		struct {
			// bitmask of channels to output (LSB = channel 0)
			unsigned int channelBitmask;
		} send;
	} payload;

	// userdata passed to callback
	void *cbContext1, *cbContext2;
	// function to call on completion of the operation (optional)
	void (*callback)(void *, void *);
} output_message_t;

#pragma GCC diagnostic pop



#endif /* LEDOUT_OUTPUTTASKPRIVATE_H_ */
