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
	kIGMPMessageLeaveGroup					= 0x17,
	kIGMPv3MessageMembershipQuery			= 0x22,
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

/**
 * Values for the `type` field of the igmpv3_packet_ipv4_record_t struct.
 */
typedef enum {
	kIGMPv3TypeModeIsInclude				= 0x01,
	kIGMPv3TypeModeIsExclude				= 0x02,

	kIGMPv3TypeChangeToInclude			= 0x03,
	kIGMPv3TypeChangeToExclude			= 0x04,
} igmpv3_packet_ipv4_record_type_t;

/**
 * IGMPv3 packet
 */
typedef struct __attribute__((__packed__)) {
	// type of record
	uint8_t type;
	// length of auxiliary data
	uint8_t auxDataLen;

	// number of attached sources
	uint16_t numSources;
	// address
	stack_ipv4_addr_t addr;

	// source addresses
	stack_ipv4_addr_t sources[];
} igmpv3_packet_ipv4_record_t;

typedef struct __attribute__((__packed__)) {
	// request type
	uint8_t type;
	// timeout, in 1/10th of a second
	uint8_t timeout;

	// checksum value
	uint16_t checksum;

	// flags
	uint16_t reserved;
	// number of sources to follow
	uint16_t numRecords;

	// source addresses
	igmpv3_packet_ipv4_record_t records[];
} igmpv3_packet_ipv4_t;


typedef enum {
	/**
	 * Sends a membership report for the given address, indicating that we
	 * are a member of that group.
	 */
	kIGMPSendMembershipForGroup				= 1,//!< kIGMPSendMembershipForGroup
	/**
	 * Sends a leave request for the given address.
	 */
	kIGMPSendLeaveGroup,               //!< kIGMPSendLeaveGroup
} igmp_task_message_type_t;

/**
 * Messages passed to the IGMP task.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

typedef struct {
	// message type
	igmp_task_message_type_t type;

	// address of the group
	stack_ipv4_addr_t address;
} igmp_task_message_t;

#pragma GCC diagnostic pop

#endif /* NET_IP_IGMPPRIVATE_H_ */
