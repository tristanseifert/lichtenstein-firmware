/*
 * IPv4.cpp
 *
 *  Created on: Feb 25, 2018
 *      Author: tristan
 */
#define LOG_MODULE "IPv4"

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include "ICMP.h"
#include "UDP.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>


// enable to log information when packets are received
#define DEBUG_PACKET_RECEPTION				0
// enable to log information when packets are transmitted
#define DEBUG_PACKET_TRANSMISSION			0


namespace ip {

/**
 * Initializes the IPv4 component.
 */
IPv4::IPv4(Stack *_s) : stack(_s) {
	// clear multicast filter
	memset(this->multicastFilter, 0, sizeof(this->multicastFilter));
	memset(this->multicastFilterRefCount, 0, sizeof(this->multicastFilterRefCount));

	// set up protocol handlers
	this->icmp = new ICMP(_s, this);
	this->udp = new UDP(_s, this);
}

/**
 * De-allocates any initialized protocol handlers.
 */
IPv4::~IPv4() {
	// deallocate protocol handlers
	delete this->icmp;
	delete this->udp;
}



/**
 * Handles a received IPv4 packet.
 */
void IPv4::handleIPv4Frame(void *_packet) {
	stack_rx_packet_t *rxPacket = (stack_rx_packet_t *) _packet;

	// make sure the version is 4
	net_ipv4_packet_t *_ipHeader = (net_ipv4_packet_t *) rxPacket->payload;
	uint8_t version = (_ipHeader->version & NET_IPV4_VERSION_MASK) >> 4;

	if(version != NET_IPV4_VERSION_4) {
		// we should never have to deal with this
		LOG(S_ERROR, "Received IPv4 frame with version %u", version);

		this->stack->doneWithRxPacket(rxPacket);
		return;
	}

	// create an RX packet
	stack_ipv4_rx_packet_t *rx;
	rx = (stack_ipv4_rx_packet_t *) pvPortMalloc(sizeof(stack_ipv4_rx_packet_t));

	if(!rx) {
		// if we couldn't allocate an RX struct, return the packet to the stack
		LOG(S_ERROR, "Couldn't allocate RX buffer");

		this->stack->doneWithRxPacket(rxPacket);
		return;
	}

	rx->stackBuffer = rxPacket;
	rx->ipv4Header = (net_ipv4_packet_t *) rxPacket->payload;
	this->packetNetworkToHost(rx->ipv4Header);

	size_t headerLength = (rx->ipv4Header->version & NET_IPV4_IHL_MASK) * 4;
	rx->payload = ((uint8_t *) rxPacket->payload) + headerLength;


#if DEBUG_PACKET_RECEPTION
	// debugging
	char srcIp[16], destIp[16];
	Stack::ipToString(rx->ipv4Header->source, srcIp, 16);
	Stack::ipToString(rx->ipv4Header->dest, destIp, 16);
#endif


	// check if the destination IP matches our IP
	if(rx->ipv4Header->dest == this->stack->getIPAddress()) {
#if DEBUG_PACKET_RECEPTION
		LOG(S_DEBUG, "Received unicast from %s to %s, protocol 0x%02x", srcIp, destIp, rx->ipv4Header->protocol);
#endif

		// call into the appropriate protocol handler
		switch(rx->ipv4Header->protocol) {
			// handle unicasted ICMP
			case kIPv4ProtocolICMP:
				this->icmp->processUnicastFrame(rx);
				break;

			// handle unicasted UDP
			case kIPv4ProtocolUDP:
				this->udp->processUnicastFrame(rx);
				break;

			// unhandled protocol
			default:
				this->releaseRxBuffer(rx);
				break;
		}
	}
	// is it a broadcast message?
	else if(isIPv4Broadcast(rx->ipv4Header->dest)) {
#if DEBUG_PACKET_RECEPTION
		LOG(S_DEBUG, "Received broadcast from %s to %s, protocol 0x%02x", srcIp, destIp, rx->ipv4Header->protocol);
#endif

		// call into the appropriate protocol handler
		switch(rx->ipv4Header->protocol) {
			// handle broadcasted UDP
			case kIPv4ProtocolUDP:
				this->udp->processBroadcastFrame(rx);
				break;

			// unhandled protocol
			default:
				this->releaseRxBuffer(rx);
				break;
		}
	}
	// lastly, is it a multicast message?
	else if(isIPv4Multicast(rx->ipv4Header->dest)) {
		// check if we've registered for this multicast address
		if(this->isMulticastAddressAllowed(rx->ipv4Header->dest)) {
#if DEBUG_PACKET_RECEPTION
			LOG(S_DEBUG, "Received multicast from %s to %s, protocol 0x%02x", srcIp, destIp, rx->ipv4Header->protocol);
#endif

			// call into the appropriate protocol handler
			switch(rx->ipv4Header->protocol) {
				// handle multicasted UDP
				case kIPv4ProtocolUDP:
					this->udp->processMulticastFrame(rx);
					break;

				// unhandled protocol
				default:
					this->releaseRxBuffer(rx);
					break;
			}
		} else {
			// ignore the frame
			this->releaseRxBuffer(rx);
		}
	}
}



/**
 * Checks whether the multicast filter list contains the given address.
 */
bool IPv4::isMulticastAddressAllowed(stack_ipv4_addr_t addr) {
	// iterate over the entire buffer
	for(size_t i = 0; i < IPv4::multicastFilterSize; i++) {
		// does the address match?
		if(this->multicastFilter[i] == addr) {
			return true;
		}
	}

	// if we get down here, assume it's not in the filter
	return false;
}

/**
 * Adds the specified IPv4 address to the multicast filter, if there is
 * space available.
 *
 * @return 0 if successful.
 */
int IPv4::addMulticastAddress(stack_ipv4_addr_t addr) {
	// check if we've already added this address
	for(size_t i = 0; i < IPv4::multicastFilterSize; i++) {
		// does the address match? if so, increment the reference count.
		if(this->multicastFilter[i] == addr) {
			this->multicastFilterRefCount[i]++;
			return 0;
		}
	}

	// try to find an empty entry (all zero's address)
	for(size_t i = 0; i < IPv4::multicastFilterSize; i++) {
		// is the address empty?
		if(this->multicastFilter[i] == kIPv4AddressZero) {
			// if so, write the address into it and set reference count
			this->multicastFilter[i] = addr;
			this->multicastFilterRefCount[i] = 1;

			return 0;
		}
	}

	// if we got down here, there wasn't space
	return -1;
}

/**
 * Removes the specified multicast address from the filter, if it exists.
 *
 * @return 0 if it was successfully removed.
 */
int IPv4::removeMulticastAddress(stack_ipv4_addr_t addr) {
	// try to find an empty entry (all zero's address)
	for(size_t i = 0; i < IPv4::multicastFilterSize; i++) {
		// does this address match?
		if(this->multicastFilter[i] == addr) {
			// decrement the reference counter
			if(this->multicastFilterRefCount[i]-- == 0)  {
				// if its zero, we can delete it from the filter
				this->multicastFilter[i] = kIPv4AddressZero;

				// TODO: shift everything below this address up one slice?
			}
			return 0;
		}
	}

	// if we got down here, the address wasn't found
	return -1;
}



/**
 * Attempts to allocate an IPv4 transmit buffer, pre-populated with the payload
 * length and protocol.
 *
 * @note This call will block, so it shouldn't be called from within the packet
 * receive pipeline.
 *
 * @return A pointer to an opaque IPv4 buffer if successful, nullptr otherwise.
 */
void *IPv4::getIPv4TxBuffer(size_t payloadLength, uint8_t protocol) {
	stack_tx_packet_t *tx;

	// request a transmit buffer
	size_t totalLength = sizeof(net_ipv4_packet_t);
	totalLength += payloadLength;

	tx = (stack_tx_packet_t *) this->stack->getTxPacket(totalLength);

	if(tx == nullptr) {
		LOG(S_ERROR, "Couldn't get TX buffer for IPv4 packet!");
		return nullptr;
	}

	// try to allocate an IPv4 TX packet
	stack_ipv4_tx_packet_t *packet;
	packet = (stack_ipv4_tx_packet_t *) pvPortMalloc(sizeof(stack_ipv4_tx_packet_t));

	if(packet == nullptr) {
		LOG(S_ERROR, "Couldn't allocate tx struct for IPv4 packet!");

		// deallocate buffer without sending it
		this->stack->discardTXPacket(tx);
		return nullptr;
	}

	// set up the packet
	packet->payloadLength = payloadLength;
	packet->sourceAddrSet = false;

	packet->stackBuffer = tx;

	packet->ipv4Header = (net_ipv4_packet_t *) tx->payload;
	packet->payload = ((uint8_t *) tx->payload) + sizeof(net_ipv4_packet_t);

	memset(packet->ipv4Header, 0, sizeof(net_ipv4_packet_t));

	// version 4, header length of 5 4-byte words
	packet->ipv4Header->version = (NET_IPV4_VERSION_4 << 4) & NET_IPV4_VERSION_MASK;
	packet->ipv4Header->version |= (5) & NET_IPV4_IHL_MASK;

	// set the length (header size plus payload)
	packet->ipv4Header->length = (uint16_t) (sizeof(net_ipv4_packet_t) + payloadLength);

	// checksum is computed by the MAC
	packet->ipv4Header->headerChecksum = 0x0000;

	// copy protocol and set TTL to default
	packet->ipv4Header->protocol = protocol;
	packet->ipv4Header->ttl = IPv4::defaultTTL;

	// clear any fragment flags and set ID
	packet->ipv4Header->id = this->currentPacketID++;
	packet->ipv4Header->fragmentFlags = 0;

	// done!
	return packet;
}

/**
 * Transmits a previously allocated IPv4 buffer.
 *
 * @note This call will block if the IPv4 address is not present in the ARP
 * cache, so it should not be called from the receive pipeline.
 */
bool IPv4::transmitIPv4TxBuffer(void *_buffer) {
	bool success;
	stack_ipv4_tx_packet_t *packet = (stack_ipv4_tx_packet_t *) _buffer;

	// parameter checking
	if(_buffer == nullptr) {
		return false;
	}

#if DEBUG_PACKET_TRANSMISSION
	// debug logging
	char destIpStr[16];
	Stack::ipToString(packet->ipv4Header->dest, destIpStr, 16);

	LOG(S_DEBUG, "Transmitting packet to %s (length %u, proto 0x%02x)", destIpStr,
			packet->payloadLength, packet->ipv4Header->protocol);
#endif

	// if the source IP hasn't been set, fill in ours now
	if(!packet->sourceAddrSet) {
		packet->ipv4Header->source = this->stack->getIPAddress();
		packet->sourceAddrSet = true;
	}

	// convert the destination IP address to a MAC address
	stack_mac_addr_t destMAC = kMACAddressInvalid;
	success = stack->resolveIPToMAC(packet->ipv4Header->dest, &destMAC);

	if(!success) {
#if DEBUG_PACKET_TRANSMISSION
		LOG(S_DEBUG, "Couldn't resolve IP %s to MAC", destIpStr);
#endif

		// if we couldn't find a MAC address, discard the packet
		this->stack->discardTXPacket(packet->stackBuffer);
		return false;
	}

#if DEBUG_PACKET_TRANSMISSION
	char destMACStr[18];
	Stack::macToString(destMAC, destMACStr, 18);

	LOG(S_DEBUG, "Sending packet for %s to %s", destIpStr, destMACStr);
#endif

	// swap byte order and transmit
	this->packetHostToNetwork(packet->ipv4Header);
	this->stack->sendTxPacket(packet->stackBuffer, destMAC, kProtocolIPv4);

	// deallocate the buffer struct we created
	vPortFree(_buffer);

	// if we get down here, assume success
	return true;
}

/**
 * Sets the destination IP address for the IP packet.
 */
void IPv4::setIPv4Destination(void *_buffer, stack_ipv4_addr_t addr) {
	stack_ipv4_tx_packet_t *packet = (stack_ipv4_tx_packet_t *) _buffer;

	packet->ipv4Header->dest = addr;
}

/**
 * Sets the source IP address for the IP packet.
 */
void IPv4::setIPv4Source(void *_buffer, stack_ipv4_addr_t addr) {
	stack_ipv4_tx_packet_t *packet = (stack_ipv4_tx_packet_t *) _buffer;

	packet->ipv4Header->source = addr;
	packet->sourceAddrSet = true;
}



/**
 * Releases a receive buffer.
 */
void IPv4::releaseRxBuffer(void *_packet) {
	stack_ipv4_rx_packet_t *packet = (stack_ipv4_rx_packet_t *) _packet;

	// return the RX buffer to the stack
	this->stack->doneWithRxPacket(packet->stackBuffer);

	// free the packet structure itself
	vPortFree(_packet);
}

/**
 * Returns the source address of this packet.
 */
stack_ipv4_addr_t IPv4::getRxBufferSource(void *_packet) {
	stack_ipv4_rx_packet_t *packet = (stack_ipv4_rx_packet_t *) _packet;

	return packet->ipv4Header->source;
}

/**
 * Returns the destination address of this packet.
 */
stack_ipv4_addr_t IPv4::getRxBufferDestination(void *_packet) {
	stack_ipv4_rx_packet_t *packet = (stack_ipv4_rx_packet_t *) _packet;

	return packet->ipv4Header->dest;
}

/**
 * Returns the length of the payload of a received packet.
 */
size_t IPv4::getRxBufferPayloadLength(void *_packet) {
	stack_ipv4_rx_packet_t *packet = (stack_ipv4_rx_packet_t *) _packet;

	size_t headerLength = (packet->ipv4Header->version & NET_IPV4_IHL_MASK) * 4;
	return (packet->ipv4Header->length - headerLength);
}



/**
 * Converts the multi-byte fields between the different endiannesses. In short,
 * this simply swaps the order of the bytes. So, care must be taken to only
 * call this function once when the packet is received before any of its fields
 * are to be accessed, or only once after a packet's fields have been
 * completely written to.
 */
void IPv4::convertPacketByteOrder(void *_packet) {
	net_ipv4_packet_t *ipv4 = (net_ipv4_packet_t *) _packet;

	ipv4->length = __builtin_bswap16(ipv4->length);
	ipv4->id = __builtin_bswap16(ipv4->id);
	ipv4->fragmentFlags = __builtin_bswap16(ipv4->fragmentFlags);
	ipv4->headerChecksum = __builtin_bswap16(ipv4->headerChecksum);
}

} /* namespace ip */
