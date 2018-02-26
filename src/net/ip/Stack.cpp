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

#include "../Network.h"

#include <LichtensteinApp.h>

#include <cstring>

namespace ip {

/**
 * Initializes the TCP/IP stack.
 */
Stack::Stack(Network *n) : net(n) {
	// allocate the protocol handlers
	this->arp = new ARP(this);
	this->ipv4 = new IPv4(this);

	// XXX: testing
//	this->ip = __builtin_bswap32(0xac100d96);
	this->ip = __builtin_bswap32(0xc0a800c8);
	this->netMask = __builtin_bswap32(0xFFFFFF00);
//	this->routerIp = __builtin_bswap32(0xac100d01);

	char ip[16];
	Stack::ipToString(this->ip, ip, 16);
	char mask[16];
	Stack::ipToString(this->netMask, mask, 16);
	char router[16];
	Stack::ipToString(this->routerIp, router, 16);

	LOG(S_DEBUG, "IP configuration: %s, netmask %s, router %s", ip, mask, router);
}

/**
 * Tears down the TCP/IP stack.
 */
Stack::~Stack() {
	// delete protocol handlers
	delete this->arp;
	delete this->ipv4;
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

	}
	// otherwise, we lost the link :(
	else {
		// clear ARP cache
		this->arp->clearARPCache();
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
 * Handles a received Ethernet packet.
 */
void Stack::receivedPacket(void *data, size_t length, uint32_t userData) {
	// if the link isn't up, ignore the packet
	if(!this->linkUp) {
		this->net->releaseRxPacket(userData);
		return;
	}

	// allocate a packet descriptor to use and initialize it
	stack_rx_packet_t *packet;
	packet = (stack_rx_packet_t *) pvPortMalloc(sizeof(stack_rx_packet_t));

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
void *Stack::getTxPacket(size_t length) {
	// verify the length isn't too big
	if(length > Stack::maxPayloadLength) {
		LOG(S_ERROR, "Maximum payload length exceeded for %u bytes", length);
		return nullptr;
	}

	// calculate total length to send on the wire
	size_t totalLength = length;
	totalLength += sizeof(stack_802_3_header_t);
//	totalLength += 4; // CRC at the end (inserted by MAC)

	void *txBuffer = this->net->getTxBuffer(totalLength);

	if(txBuffer == nullptr) {
		return nullptr;
	}

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

	// fill in the source and destination MAC fields
	stack_802_3_header_t *ethHeader = (stack_802_3_header_t *) packet->txBuffer;

	ethHeader->macSrc = this->mac;
	ethHeader->macDest = destination;
	ethHeader->etherType = __builtin_bswap16(proto);

	// XXX: debugging
	char macSrc[18], macDest[18];
	Stack::macToString(ethHeader->macSrc, macSrc, 18);
	Stack::macToString(ethHeader->macDest, macDest, 18);
	LOG(S_DEBUG, "Sending ethernet frame from %s to %s, type 0x%04x", macSrc, macDest, proto);

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
	// is the address a broadcast address?
	if(isIPv4Broadcast(addr)) {
		*result = kMACAddressBroadcast;
		return true;
	}
	// is the address a multicast address?
	else if(isIPv4Multicast(addr)) {
		// TODO: convert multicast address

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
