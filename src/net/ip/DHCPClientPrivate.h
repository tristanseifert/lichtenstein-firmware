/*
 * DHCPClientPrivate.h
 *
 *  Created on: Mar 4, 2018
 *      Author: tristan
 */

#ifndef NET_IP_DHCPCLIENTPRIVATE_H_
#define NET_IP_DHCPCLIENTPRIVATE_H_

#include "StackTypes.h"

#include <cstdint>

// port number we receive DHCP responses on
#define DHCP_CLIENT_PORT					68
// port number we send DHCP messages to
#define DHCP_SERVER_PORT					67



// DHCP magic cookie value
#define DHCP_COOKIE							0x63825363



/**
 * DHCP options
 */
enum {
	kDHCPOptionPadding						= 0x00,
	kDHCPOptionEnd							= 0xFF,

	kDHCPOptionRequestedAddress				= 50,
	kDHCPOptionMessageType					= 53,
	kDHCPOptionLeaseTime						= 51,
	kDHCPOptionServerAddress					= 54,
	kDHCPOptionRequestedOptions				= 55,

	kDHCPOptionSubnetMask					= 1,
	kDHCPOptionRouter						= 3,
	kDHCPOptionDNS							= 6,
	kDHCPOptionsSyslogServer					= 7,
	kDHCPOptionsTFTPServer					= 66,
	kDHCPOptionsBootFileName					= 67,
	kDHCPOptionsHostname						= 12,
	kDHCPOptionsNTPServer					= 42,
	kDHCPOptionsUTCOffset					= 2,
};



/**
 * DHCP opcodes
 */
enum {
	kDHCPOpCodeRequest						= 1,
	kDHCPOpCodeReply							= 2,
};

/**
 * Values for the DHCP Message Type (option 53) option
 */
enum {
	kDHCPMessageTypeDiscover				= 1,
	kDHCPMessageTypeOffer					= 2,
	kDHCPMessageTypeRequest					= 3,
	kDHCPMessageTypeDecline					= 4,
	kDHCPMessageTypeAck						= 5,
	kDHCPMessageTypeNack					= 6,
	kDHCPMessageTypeRelease					= 7,
	kDHCPMessageTypeInform					= 8,
};

/**
 * Hardware types
 */
enum {
	kDHCPHardwareTypeEthernet				= 1,
};

/**
 * Flags
 */
enum {
	kDHCPFlagBroadcast						= (1 << 15)
};

/**
 * DHCP packet
 */
typedef struct __attribute__((__packed__)) {
	// op code
	uint8_t op;

	// HTYPE (hardware address type)
	uint8_t htype;
	// HLEN (length of hardware address)
	uint8_t hlen;
	// hops (???)
	uint8_t hops;

	// transaction id
	uint32_t xid;
	// seconds since the start of lease acquisition
	uint16_t secs;
	// flags
	uint16_t flags;

	// client IP address
	stack_ipv4_addr_t clientAddr;
	// IP address assigned by server
	stack_ipv4_addr_t yourAddr;
	// IP address of the server: client should send replies there
	stack_ipv4_addr_t replyAddr;
	// DHCP gateway address (NOT default router!)
	stack_ipv4_addr_t gatewayAddr;

	// MAC address
	union {
		stack_mac_addr_t mac;
		uint8_t padding[16];
	} hardwareAddr;

	// server name sent in DHCPOFFER and DHCPACK
	char bootpServerName[64];
	// boot file name sent in DHCPOFFER
	char bootFileName[128];

	// magic cookie value
	uint32_t cookie;

	// start of DHCP options
	uint8_t options[];
} dhcp_packet_ipv4_t;

#endif /* NET_IP_DHCPCLIENTPRIVATE_H_ */
