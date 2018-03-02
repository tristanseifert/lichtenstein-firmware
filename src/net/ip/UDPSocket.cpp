/*
 * UDPSocket.cpp
 *
 *  Created on: Mar 2, 2018
 *      Author: tristan
 */
#define LOG_MODULE "UDP"

#include "UDPSocket.h"

#define UDP_PRIVATE 1
#include "UDPPrivate.h"
#include "UDP.h"

#include "Stack.h"
#include "StackPrivate.h"

#include <LichtensteinApp.h>

namespace ip {

/**
 * Initializes an UDP socket.
 */
UDPSocket::UDPSocket(UDP *_udp) : udp(_udp) {

}

/**
 * Removes any pending descriptors from the socket.
 */
UDPSocket::~UDPSocket() {

}

} /* namespace ip */
