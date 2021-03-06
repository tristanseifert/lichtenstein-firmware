/*
 * NetworkPrivate.h
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */

#ifndef NET_NETWORKPRIVATE_H_
#define NET_NETWORKPRIVATE_H_

#include <cstdint>
#include <cstddef>

/**
 * Types of network messages.
 */
typedef enum {
	kNetworkMessageUnknown,

	/**
	 * A frame was received from the MAC, and should be processed.
	 *
	 * All fields are valid.
	 *
	 * @note The frame must be released when the stack no longer needs it.
	 */
	kNetworkMessageReceivedFrame			= 1000,

	/**
	 * A previously queued frame has been transmitted. The DMA buffer should
	 * be returned to the CPU and can be used for the next packet.
	 *
	 * index is the only valid field.
	 */
	kNetworkMessageTransmittedFrame,

	/**
	 * The link state was changed.
	 *
	 * index is the only valid field, and it corresponds to the current link
	 * state: 0 if the link is down, 1 if it is up.
	 */
	kNetworkMessageLinkStateChanged,


	/**
	 * A packet was lost because no receive buffers were available.
	 *
	 * If the index value is 1, the receive process has been completely
	 * stopped.
	 */
	kNetworkMessageRxPacketLost,


	/**
	 * Unknown interrupt: the status register is written to the index reg.
	 */
	kNetworkMessageDebugUnknownIRQ
} network_message_type_t;

/**
 * Message buffer passed to the network task
 */
typedef struct __attribute__((__packed__)) {
	// type of the message: this defines what the other values mean
	network_message_type_t type;

	// index of the packet buffer, if relevant
	unsigned int index;
	// buffer to data of the packet, if relevant
	uint8_t *data;
	// length of the packet, if relevant
	size_t packetLength;

	// additional user data
	unsigned int userData;
} network_message_t;

#endif /* NET_NETWORKPRIVATE_H_ */
