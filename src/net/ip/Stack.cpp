/*
 * Stack.cpp
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */
#define LOG_MODULE "STK"

#include "Stack.h"
#include "StackPrivate.h"

#include "ARP.h"

#include "IPv4.h"
#include "UDP.h"
#include "DHCPClient.h"

#include "../Network.h"

#include <LichtensteinApp.h>
#include "../board/Board.h"

#include <cstring>



// produce logging output when resolving IP addresses to MAC addresses
#define LOG_ADDRESS_TO_MAC						0
// produce logging output for sending frames
#define LOG_TRANSMIT								0
// produce logging output for received frames
#define LOG_RECEIVE								0

// Use a static IP (only in debugging)
#define TEST_STATIC								0


namespace ip {

/**
 * Initializes the TCP/IP stack.
 */
Stack::Stack(Network *n) : net(n) {
	// allocate the protocol handlers
	this->arp = new ARP(this);
	this->ipv4 = new IPv4(this);

	// create DHCP client
	this->dhcp = new DHCPClient(this);

	// XXX: testing
#ifdef DEBUG
#if TEST_STATIC
	this->ip = __builtin_bswap32(0xc0a800c8); // 192.168.0.200
	this->netMask = __builtin_bswap32(0xFFFFFF00);
//	this->routerIp = __builtin_bswap32(0xac100d01);

	this->useDHCP = false;
#else
	this->useDHCP = false;
#endif // TEST_STATIC
#endif // DEBUG

	// initialize default hostname
	this->setHostname("lichtenstein");
}

/**
 * Tears down the TCP/IP stack.
 */
Stack::~Stack() {
	// delete protocol handlers
	delete this->arp;
	delete this->ipv4;

	// delete DHCP client
	if(this->dhcp) {
		delete this->dhcp;
	}
}


/**
 * Sets our Unicast MAC address. Packets will be checked to ensure that they
 * have this MAC address as the destination before any processing is done.
 */
void Stack::setUnicastMACAddress(uint8_t *src) {
	memcpy(this->mac.bytes, src, 6);
}

/**
 * Called whenever the state of the link changes.
 */
void Stack::linkStateChanged(bool _linkUp) {
	// return if the link state didn't change
	if(this->linkUp == _linkUp) {
		return;
	}

	this->linkUp = _linkUp;

	// did the link come up?
	if(linkUp) {
		// if using DHCP, start request
		if(this->useDHCP) {
			this->dhcp->requestIP();
		}
		// otherwise we have a static IP
		else {
			this->ipConfigBecameValid();
		}
	}
	// otherwise, we lost the link :(
	else {
		// reset DHCP state machine
		if(this->useDHCP) {
			this->dhcp->reset();
		}

		// invalidate IP configuration
		this->isIPv4ConfigValid = false;

		// clear ARP cache
		this->arp->clearARPCache();
	}
}

/**
 * Changes the DHCP state. When enabling DHCP, this will immediately begin
 * the process of acquiring an IP address.
 *
 * @param status New DHCP state
 */
void Stack::setUsesDHCP(bool status) {
	// if we already use DHCP and we call it again, re-
	if(this->useDHCP == status) {
		return;
	}

	// if we're not using DHCP right now, activate it
	if(this->useDHCP == false && status == true) {
		LOG(S_DEBUG, "Enabling DHCP");

		// reset DHCP client
		this->dhcp->reset();

		// if link is up, request a lease
		if(this->linkUp) {
			this->dhcp->requestIP();
		}
	}

	// change the variable and kill the DHCP client if needed
	this->useDHCP = status;

	if(this->useDHCP == false) {
		this->dhcp->reset();
	}
}

/**
 * Returns our IP address. This will be all zeroes if no IP has been configured
 * yet.
 */
stack_ipv4_addr_t Stack::getIPAddress(void) const {
	return this->ip;
}

/**
 * Sets the hostname. This isn't really used outside of what the DHCP client
 * will send when it requests a lease.
 *
 * @param name New hostname
 *
 * TODO: figure out why this sometimes overwrites the stack itself
 */
void Stack::setHostname(const char *name) {
	// free old hostname buffer
	if(this->hostname) {
		vPortFree(this->hostname);
	}

	// allocate buffer for new hostname
	size_t len = strlen(name) + 1;

	this->hostname = (char *) pvPortMalloc(len);

	if(this->hostname == nullptr) {
		LOG(S_ERROR, "Couldn't allocate hostname buffer");
		return;
	}

	memset(this->hostname, 0, len);

//	LOG(S_DEBUG, "Setting hostname to '%s'; this = 0x%08x, ptr = 0x%08x", name, this, this->hostname);

	// copy the hostname
	strncpy(this->hostname, name, len);

	LOG(S_DEBUG, "Changed hostname: %s", this->hostname);
}



/**
 * Called after the IPv4 configuration becomes valid. This is usually called
 * once we've received a DHCP lease, or on link up if the IP is static.
 */
void Stack::ipConfigBecameValid(void) {
	// mark the IP config as valid
	this->isIPv4ConfigValid = true;

	// print info
	char ipStr[16], mask[16], router[16];
	Stack::ipToString(this->ip, ipStr, 16);
	Stack::ipToString(this->netMask, mask, 16);
	Stack::ipToString(this->routerIp, router, 16);

	LOG(S_INFO, "IP configuration: %s, netmask %s, router %s", ipStr, mask, router);

	// send a gratuitous ARP
	this->arp->sendGratuitousARP();
}



/**
 * Allocates an UDP socket.
 */
UDPSocket *Stack::createUDPSocket(void) {
	return this->ipv4->udp->createSocket();
}



/**
 * Handles a received Ethernet packet.
 */
void Stack::receivedPacket(void *data, size_t length, uint32_t userData) {
	// if the link isn't up, ignore the packet
	if(!this->linkUp) {
		this->net->releaseRxPacket(userData);
		return;
	}

	// handle zero-length packets
	if(length == 0) {
		stack_802_3_header_t *header = (stack_802_3_header_t *) data;
		LOG(S_INFO, "Received zero-length packet; type 0x%x", header->etherType);

		this->net->releaseRxPacket(userData);
		return;
	}

	// toggle IP activity LED
	Board::sharedInstance()->toggleLED(Board::kBoardLEDIPAct);

	// allocate a packet descriptor to use and initialize it
	stack_rx_packet_t *packet;
	packet = (stack_rx_packet_t *) pvPortMalloc(sizeof(stack_rx_packet_t));

	if(packet == nullptr) {
		// return if we couldn't allocate one (BAD)
		LOG(S_ERROR, "Couldn't allocate RX packet");

		this->net->releaseRxPacket(userData);
		return;
	}

	memset(packet, 0, sizeof(stack_rx_packet_t));

	packet->length = length;
	packet->origBuf = data;
	packet->userData = userData;

	packet->l2Proto = kProtocolUnknown;
	packet->ipv4Proto = kIPv4ProtocolUnknown;

	// byte-swap values as needed
	stack_802_3_header_t *header = (stack_802_3_header_t *) data;
	packet->ethHeader = header;
	packet->payload = header->payload;

	header->etherType = __builtin_bswap16(header->etherType);

	// if the packet has length in place of EtherType, discard it
	if(header->etherType < 1536) {
		LOG(S_ERROR, "Got Ethernet I packet, ignoring");

		this->doneWithRxPacket(packet);
		return;
	}

	// copy the EtherType value
	packet->l2Proto = (stack_protocol_l2_t) header->etherType;

	// is this a multicast/broadcast frame?
	if(header->macDest.bytes[0] & 0b00000001) {
		// if so, clear the unicast flag
		packet->unicast = false;
	} else {
		// otherwise, it has to be an unicast packet
		packet->unicast = true;
	}

	// logging
#if LOG_RECEIVE
	char macStr[18];
	Stack::macToString(packet->ethHeader->macSrc, macStr, 18);
	LOG(S_DEBUG, "Received packet from %s", macStr);
#endif


	// run the appropriate handler for the L2 protocol
	switch(packet->l2Proto) {
		// call into ARP handler
		case kProtocolARP:
			this->arp->handleARPFrame(packet);
			break;

		// call into IPv4 handler
		case kProtocolIPv4:
			this->ipv4->handleIPv4Frame(packet);
			break;

		// IPv6 is currently unimplemented
		case kProtocolIPv6:
			this->doneWithRxPacket(packet);
			break;

		// unknown protocol
		default:
			LOG(S_WARN, "Unknown EtherType: 0x%04x", packet->l2Proto);
			this->doneWithRxPacket(packet);
			break;
	}
}

/**
 * When the upper layer protocols have finished inspecting a particular packet,
 * this method is called. This de-allocates the memory for the packet struct,
 * and releases the receive buffer.
 */
void Stack::doneWithRxPacket(void *_inPacket) {
	// cast packet
	stack_rx_packet_t *packet = (stack_rx_packet_t *) _inPacket;

	// release RX packet
	this->net->releaseRxPacket(packet->userData);

	// de-allocate its memory
	vPortFree(packet);
}



/**
 * Attempts to acquire a transmit buffer with the specified payload length.
 *
 * The length is added to the length of the Ethernet header.
 *
 * @note This is a blocking call, so it shouldn't be called from within the
 * receive pipeline.
 */
void *Stack::getTxPacket(size_t length, int timeout) {
	// verify the length isn't too big
	if(length > Stack::maxPayloadLength) {
		LOG(S_ERROR, "Maximum payload length exceeded for %u bytes", length);
		return nullptr;
	}

	// handle negative timeouts
	if(timeout < 0) {
		timeout = portMAX_DELAY;
	}

	// calculate total length to send on the wire
	size_t totalLength = length;
	totalLength += sizeof(stack_802_3_header_t);
//	totalLength += 4; // CRC at the end (inserted by MAC)

	void *txBuffer = this->net->getTxBuffer(totalLength, timeout);

	if(txBuffer == nullptr) {
		return nullptr;
	}

	// clear the buffer
	memset(txBuffer, 0, totalLength);

	// allocate a packet
	stack_tx_packet_t *packet;
	packet = (stack_tx_packet_t *) pvPortMalloc(sizeof(stack_tx_packet_t));

	// attempt to acquire a tx buffer
	packet->txBuffer = txBuffer;
	packet->length = totalLength;

	packet->payloadLength = length;
	packet->payload = ((stack_802_3_header_t *) packet->txBuffer)->payload;

	// done!
	return packet;
}

/**
 * Discards a previously allocated TX packet without sending it.
 */
void Stack::discardTXPacket(void *_packet) {
	stack_tx_packet_t *packet = (stack_tx_packet_t *) _packet;

	// mark it s available with the network driver
	this->net->releaseTxBuffer(packet->txBuffer);

	// deallocate its memory
	vPortFree(_packet);
}

/**
 * Sends a previously configured TX packet.
 *
 * @note After this call, the packet structure is deallocated.
 */
int Stack::sendTxPacket(void *_packet, stack_mac_addr_t destination, uint16_t proto) {
	stack_tx_packet_t *packet = (stack_tx_packet_t *) _packet;

	// exit if the link is down
	if(!this->linkUp) {
		vPortFree(_packet);
		return -1;
	}

	// toggle IP activity LED
	Board::sharedInstance()->toggleLED(Board::kBoardLEDIPAct);

	// fill in the source and destination MAC fields
	stack_802_3_header_t *ethHeader = (stack_802_3_header_t *) packet->txBuffer;

	ethHeader->macSrc = this->mac;
	ethHeader->macDest = destination;
	ethHeader->etherType = __builtin_bswap16(proto);

#if LOG_TRANSMIT
	char macSrc[18], macDest[18];
	Stack::macToString(ethHeader->macSrc, macSrc, 18);
	Stack::macToString(ethHeader->macDest, macDest, 18);
	LOG(S_DEBUG, "Sending Ethernet frame from %s to %s, type 0x%04x", macSrc, macDest, proto);
#endif

	// send it!
	this->net->queueTxBuffer(packet->txBuffer);

	// we can de-allocate the packet now
	vPortFree(_packet);

	return 0;
}



/**
 * Resolves an IPv4 address to a MAC by the use of ARP.
 *
 * @note This call will block.
 */
bool Stack::resolveIPToMAC(stack_ipv4_addr_t addr, stack_mac_addr_t *result, int timeout) {
#if LOG_ADDRESS_TO_MAC
	// logging
	char ipStr[16];
	Stack::ipToString(addr, ipStr, 16);
	LOG(S_DEBUG, "Resolving IP address %s", ipStr);
#endif

	// is the address a broadcast address?
	if(isIPv4Broadcast(addr)) {
		*result = kMACAddressBroadcast;
		return true;
	}
	// is the address a multicast address?
	else if(isIPv4Multicast(addr)) {
		// get the low 23 bits of the address
		uint32_t lowBits = __builtin_bswap32(addr) & 0x007FFFFF;

		// copy multicast OUI
		result->bytes[0] = 0x01;
		result->bytes[1] = 0x00;
		result->bytes[2] = 0x5E;

		// now, copy the low 3 bytes of the IP
		result->bytes[3] = (lowBits & 0x007F0000) >> 16;
		result->bytes[4] = (lowBits & 0x0000FF00) >> 8;
		result->bytes[5] = (lowBits & 0x000000FF);

		return true;
	}
	// otherwise, perform an ARP lookup
	else {
		// is this IP in our subnet?
		if(this->isIPLocal(addr)) {
			// if so, just perform an ARP query.
			return this->arp->resolveIPv4(addr, result, timeout);
		} else {
			// if we have a router address, query for it. otherwise give up
			if(this->routerIp != kIPv4AddressZero) {
#if LOG_ADDRESS_TO_MAC
				LOG(S_DEBUG, "Address outside subnet, resolving router instead");
#endif

				return this->resolveIPToMAC(this->routerIp, result, timeout);
			} else {
				// no router address specified
				return false;
			}
		}
	}

	// if we get down here, we couldn't resolve the IP
	return false;
}



/**
 * Adds the given MAC address to the hash filter.
 *
 * @param addr MAC address to add
 */
void Stack::addMulticastMAC(stack_mac_addr_t addr) {
	this->net->mac->setMulticastAddr(&addr.bytes[0], true);
}

/**
 * Removes the given MAC address from the hash filter.
 *
 * @param addr MAC address to remove
 */
void Stack::removeMulticastMAC(stack_mac_addr_t addr) {
	this->net->mac->setMulticastAddr(&addr.bytes[0], false);
}



/**
 * Converts an IP address to a string.
 */
void Stack::ipToString(stack_ipv4_addr_t addr, char *out, size_t outLen) {
	mini_snprintf(out, outLen, "%u.%u.%u.%u", (addr & 0x000000FF),
			(addr & 0x0000FF00) >> 8, (addr & 0x00FF0000) >> 16,
			(addr & 0xFF000000) >> 24);
}

/**
 * Converts a MAC address to string.
 */
void Stack::macToString(stack_mac_addr_t addr, char *out, size_t outLen) {
	mini_snprintf(out, outLen, "%02x:%02x:%02x:%02x:%02x:%02x", addr.bytes[0],
			addr.bytes[1], addr.bytes[2], addr.bytes[3], addr.bytes[4],
			addr.bytes[5]);
}

/**
 * Checks whether the given IP is in the same subnet as we are.
 */
bool Stack::isIPLocal(stack_ipv4_addr_t addr) {
	return Stack::isIPInSubnet(addr, this->ip, this->netMask);
}

/**
 * Given a netmask, checks whether the netAddr address is in the same subnet
 * as the `addr` address.
 */
bool Stack::isIPInSubnet(stack_ipv4_addr_t addr, stack_ipv4_addr_t netAddr, stack_ipv4_addr_t netmask) {
	return ((netAddr & netmask) == (addr & netmask));
}

} /* namespace ip */
