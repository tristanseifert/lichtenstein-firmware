/*
 * ICMPPrivate.h
 *
 *  Created on: Feb 26, 2018
 *      Author: tristan
 */

#ifndef NET_IP_ICMPPRIVATE_H_
#define NET_IP_ICMPPRIVATE_H_

#include "StackTypes.h"

#include <cstdint>

/**
 * Various values for the type code of the ICMP packet.
 */
typedef enum {
	/**
	 * Echo reply: this is the type of packets sent in response to an echo
	 * request.
	 */
	kICMPTypeEchoReply						= 0,

	/**
	 * Echo request: packets received with this type will generate an echo
	 * reply, if replying is enabled.
	 */
	kCIMPTypeEchoRequest						= 8,
} icmp_packet_type_t;

/**
 * An ICMP packet, as sent across the wire.
 */
typedef struct __attribute__((__packed__)) {
	// type
	uint8_t type;
	// code
	uint8_t code;

	// checksum
	uint16_t checksum;

	// remaining arguments; these vary based on type and code
	union {
		// remainder of header: this is used to pad the struct
		uint32_t remainder;

		// echo request
		struct __attribute__((__packed__)) {
			uint16_t identifier;
			uint16_t sequence;

			uint8_t payload[];
		} echoRequest;
	} data;
} icmp_packet_ipv4_t;



/**
 * Types of task messages
 */
typedef enum {
	/**
	 * Generate a reply to an echo request. The reply should be sent to the
	 * IP specified in the payload of the message.
	 */
	kICMPMessageSendEchoReply				= 1,
} icmp_task_message_type_t;

/**
 * This describes a message that's passed to the ICMP task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// type of the message
	icmp_task_message_type_t type;

	// IP address from which we received the message
	stack_ipv4_addr_t source;

	// any additional payload data
	union {
		struct {
			// these are already in network byte order
			uint16_t identifier;
			uint16_t sequence;

			// additional payload in the packet
			void *additionalData;
			size_t additionalDataLength;
		} echoRequest;
	} data;
} icmp_task_message_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_ICMPPRIVATE_H_ */
