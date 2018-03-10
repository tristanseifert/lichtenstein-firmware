/*
 * Socket.cpp
 *
 *  Created on: Mar 1, 2018
 *      Author: tristan
 */
#define LOG_MODULE "SOCK"

#include "Socket.h"

#include "IPv4.h"

#include <LichtensteinApp.h>

namespace ip {

/**
 * Initializes the socket and any structures we need.
 */
Socket::Socket() {

}

/**
 * Cleans up any resources we allocated earlier.
 */
Socket::~Socket() {

}


/**
 * Handles setting a socket option. This handles lower-level options (such as
 * multicast groups) that are common to all higher-level protocols.
 */
int Socket::setSockOpt(socket_protocol_t protocol, socket_option_t option, void *value, size_t length) {
	int err;

	// is the option IPv4?
	if(protocol == kSocketProtocolIPv4) {
		switch(option) {
			// add multicast membership
			case kSockOptJoinMulticast: {
				// ensure value length is correct
				if(length != sizeof(stack_ipv4_addr_t)) {
					return Socket::ErrInvalidOptionLength;
				}

				// add multicast address
				stack_ipv4_addr_t addr = *((stack_ipv4_addr_t *) value);
				err = this->ipv4->addMulticastAddress(addr);

				if(err != 0) {
					return Socket::ErrMulticastError;
				} else {
					return Socket::ErrSuccess;
				}
			}

			// remove multicast membership
			case kSockOptLeaveMulticast: {
				// ensure value length is correct
				if(length != sizeof(stack_ipv4_addr_t)) {
					return Socket::ErrInvalidOptionLength;
				}

				// remove multicast address
				stack_ipv4_addr_t addr = *((stack_ipv4_addr_t *) value);
				err = this->ipv4->removeMulticastAddress(addr);

				if(err != 0) {
					return Socket::ErrMulticastError;
				} else {
					return Socket::ErrSuccess;
				}
			}

			// unhandled option
			default:
				return Socket::ErrUnimplemented;
				break;
		}
	}

	// the sockopt must be unhandled if we get here
	return Socket::ErrInvalidProtocol;
}

} /* namespace ip */
