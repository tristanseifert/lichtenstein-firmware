/*
 * StackTypes.h
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef NET_IP_STACKTYPES_H_
#define NET_IP_STACKTYPES_H_

#include <cstdint>
#include <cstring>

/**
 * MAC address
 */
typedef union __attribute__((__packed__)) {
	uint8_t bytes[6];

	struct __attribute__((__packed__)) {
		// OUI
		uint8_t oui[3];
		// device ID
		uint8_t device[3];
	} blah;
} stack_mac_addr_t;


inline bool operator==(const stack_mac_addr_t& lhs, const stack_mac_addr_t& rhs) {
	return (memcmp(&lhs, &rhs, 6) == 0);
}
inline bool operator!=(const stack_mac_addr_t& lhs, const stack_mac_addr_t& rhs) { return !(lhs == rhs); }

/**
 * Broadcast MAC address
 */
static const stack_mac_addr_t kMACAddressBroadcast = {
	.bytes = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};



/**
 * IPv4 Address, as sent over the network. This means that this value will be
 * in big-endian (e.g. network byte order).
 */
/*typedef union __attribute__((__packed__)) {
	// raw format as sent over the wire
	uint32_t address;

	// individual octets
	struct __attribute__((__packed__)) {
		uint8_t first;
		uint8_t second;
		uint8_t third;
		uint8_t fourth;
	} octets;
} stack_ipv4_addr_t;*/
typedef uint32_t stack_ipv4_addr_t;



#endif /* NET_IP_STACKTYPES_H_ */
