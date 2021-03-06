/*
 * ARPPrivate.h
 *
 * Various private structures for the ARP handler.
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef NET_IP_ARPPRIVATE_H_
#define NET_IP_ARPPRIVATE_H_

#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstdint>

/// Request frame
#define ARP_OP_REQUEST						1
/// Reply frame
#define ARP_OP_REPLY							2

/// hardware type for Ethernet
#define ARP_HW_ETHERNET						1

/**
 * ARP frame
 */
typedef struct __attribute__((__packed__)) {
	// hardware type
	uint16_t hwType;
	// protocol address type (same as EtherType)
	uint16_t protoType;

	// hardware address length, in bytes
	uint8_t hwAddressLength;
	// protocol address length, in bytes
	uint8_t protoAddressLength;

	// operation
	uint16_t op;

	// sender MAC address
	stack_mac_addr_t senderMAC;
	// sender IP address
	stack_ipv4_addr_t senderIP;

	// target MAC address
	stack_mac_addr_t targetMAC;
	// target IP address
	stack_ipv4_addr_t targetIP;
} arp_ipv4_packet_t;



/**
 * A single entry in the ARP cache.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// is this entry valid?
	bool valid;
	// age (ticks)
	uint32_t age;

	// MAC address
	stack_mac_addr_t mac;
	// IPv4 address
	stack_ipv4_addr_t ip;
} arp_ipv4_cache_entry_t;

#pragma GCC diagnostic pop



/**
 * Possible actions for the ARP task.
 */
typedef enum {
	/**
	 * Processes a received ARP message, inserting it into the cache.
	 */
	kARPMessageReceived						= 1,

	/**
	 * Formulates and sends a response to an ARP request for our MAC address.
	 */
	kARPMessageSendReply,

	/**
	 * Sends an ARP request for the given IP address.
	 */
	kARPMessageResolveIP,

	/**
	 * Transmits a gratuitous ARP.
	 */
	kARPMessageSendGratuitous
} arp_task_message_type_t;

/**
 * Message passed to the ARP task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// message type
	arp_task_message_type_t type;

	// payload data
	union {
		// used when an ARP message was received
		struct {
			// MAC address to learn
			stack_mac_addr_t mac;
			// IP to associate with the MAC
			stack_ipv4_addr_t address;
		} learn;

		// used when generating a response
		struct {
			// MAC address to which the response is addressed
			stack_mac_addr_t mac;
			// IP address to which the response is addressed
			stack_ipv4_addr_t address;
		} sendResponse;

		// used when resolving an IP not in the cache
		struct {
			// signal this semaphore upon response
			SemaphoreHandle_t completion;
			// address to resolve
			stack_ipv4_addr_t address;
		} request;
	} payload;
} arp_task_message_t;

#pragma GCC diagnostic pop

/**
 * Array of notification entries.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// only consider entries with the valid flag set
	bool valid;

	// signal this semaphore upon response
	SemaphoreHandle_t completion;
	// address to resolve
	stack_ipv4_addr_t address;
} arp_resolve_notifications_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_ARPPRIVATE_H_ */
