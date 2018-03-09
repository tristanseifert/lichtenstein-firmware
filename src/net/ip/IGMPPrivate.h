/*
 * IGMPPrivate.h
 *
 * Private types used in processing IGMP.
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */

#ifndef NET_IP_IGMPPRIVATE_H_
#define NET_IP_IGMPPRIVATE_H_

#include "StackTypes.h"

#include <cstddef>

enum {
	kIGMPMessageMembershipQuery				= 0x11,
	kIGMPMessageMembershipReport				= 0x16,
	kIGMPMessageLeaveGroup					= 0x17
};

/**
 * IGMP packet
 */
typedef struct __attribute__((__packed__)) {
	// request type
	uint8_t type;
	// timeout, in 1/10th of a second
	uint8_t timeout;

	// checksum value
	uint16_t checksum;

	// IPv4 address of the group
	stack_ipv4_addr_t address;
} igmp_packet_ipv4_t;



#endif /* NET_IP_IGMPPRIVATE_H_ */
