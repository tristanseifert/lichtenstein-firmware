/*
 * IGMP.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */
#define LOG_MODULE "ICMP"

#include "IGMP.h"
#include "IGMPPrivate.h"

#include "IPv4.h"
#include "IPv4Private.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

#include <cstddef>
#include <cstring>


namespace ip {

/**
 * Initializes the IGMP handler.
 */
IGMP::IGMP(Stack *_stack, IPv4 *_ipv4) : stack(_stack), ipv4(_ipv4) {
	// register for the multicast address

}

/**
 * Tears down the IGMP handler.
 */
IGMP::~IGMP() {

}



/**
 * Handles a received IGMP packet.
 *
 * @param _packet Address of payload (start of IGMP packet)
 */
void IGMP::processMulticastFrame(void *_packet) {
	this->receivedPackets++;

	stack_ipv4_rx_packet_t *rx = (stack_ipv4_rx_packet_t *)  _packet;

	// TODO: handle packet
	this->ipv4->releaseRxBuffer(rx);
}



/**
 * Converts the byte order of multi-byte fields in an IGMP packet.
 */
void IGMP::convertPacketByteOrder(void *_igmp) {
	igmp_packet_ipv4_t *igmp = (igmp_packet_ipv4_t *) _igmp;

	igmp->checksum = __builtin_bswap16(igmp->checksum);
}

} /* namespace ip */
