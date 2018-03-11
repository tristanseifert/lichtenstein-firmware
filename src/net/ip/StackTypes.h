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
inline bool operator!=(const stack_mac_addr_t& lhs, const stack_mac_addr_t& rhs) {
	return !(lhs == rhs);
}

/**
 * Broadcast MAC address
 */
static const stack_mac_addr_t kMACAddressBroadcast = {
	.bytes = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};
/**
 * Invalid (all zeroes) MAC address
 */
static const stack_mac_addr_t kMACAddressInvalid = {
	.bytes = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
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

/**
 * All zero's (this network broadcast) address.
 */
static const stack_ipv4_addr_t kIPv4AddressZero = __builtin_bswap32(0x00000000);

/**
 * Local broadcast address
 */
static const stack_ipv4_addr_t kIPv4AddressBroadcast = __builtin_bswap32(0xFFFFFFFF);

/**
 * Multicast address for all hosts
 */
static const stack_ipv4_addr_t kIPv4AddressAllHosts = __builtin_bswap32(0xE0000001);

/**
 * Multicast address for all routers
 */
static const stack_ipv4_addr_t kIPv4AddressAllRouters = __builtin_bswap32(0xE0000002);



/**
 * Determines if an IPv4 address is a multicast address.
 */
inline bool isIPv4Multicast(stack_ipv4_addr_t address) {
	static const stack_ipv4_addr_t multicastMask = __builtin_bswap32(0xF0000000);
	static const stack_ipv4_addr_t multicastComparison = __builtin_bswap32(0xE0000000);

	return ((address & multicastMask) == multicastComparison);
};

/**
 * Determines if an IPv4 address is a broadcast address. This will be true
 * if the address falls into either 0.0.0.0/8 or 255.255.255.255/32.
 */
inline bool isIPv4Broadcast(stack_ipv4_addr_t address) {
	static const stack_ipv4_addr_t thisMask = __builtin_bswap32(0xFF000000);
	static const stack_ipv4_addr_t thisComparison = __builtin_bswap32(0x00000000);

	static const stack_ipv4_addr_t broadcastAddress = __builtin_bswap32(0xFFFFFFFF);

	// is it in 0.0.0.0/8 (all stations on current network)
	if((address & thisMask) == thisComparison) {
		return true;
	}

	// is the address 255.255.255.255?
	if(address == broadcastAddress) {
		return true;
	}

	// it's not a broadcast address otherwise
	return false;
}


#endif /* NET_IP_STACKTYPES_H_ */
