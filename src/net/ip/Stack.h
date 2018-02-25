/*
 * Stack.h
 *
 * Basic class for the TCP/IP stack.
 *
 *  Created on: Feb 24, 2018
 *      Author: tristan
 */

#ifndef STACK_H_
#define STACK_H_

#include <cstddef>
#include <cstdint>

#include "StackTypes.h"

#include "../EthMAC.h"

class Network;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class ARP;

	class Stack {
		public:
			Stack(Network *n);
			virtual ~Stack();

		// API for configuring the Ethernet and IP properties
		public:
			void setUnicastMACAddress(uint8_t *src);

			void linkStateChanged(bool linkUp);

			stack_ipv4_addr_t getIPAddress(void) const;

		// API for receiving/transmitting packets
		public:
			void receivedPacket(void *data, size_t length, uint32_t userData);

		// private calls for packet handling
		private:
			void doneWithRxPacket(void *);


			void *getTxPacket(size_t);

			int sendTxPacket(void *, stack_mac_addr_t, uint16_t);
			inline int broadcastTxPacket(void *tx, uint16_t type) {
				return this->sendTxPacket(tx, kMACAddressBroadcast, type);
			}

		// protocol handlers
		private:
			friend class ARP;

			ARP *arp = nullptr;

		// helpers
		public:
			static void ipToString(stack_ipv4_addr_t addr, char *out, size_t outLen);
			static void macToString(stack_mac_addr_t addr, char *out, size_t outLen);

		private:
			// network handler (used to rx/tx packets)
			Network *net = nullptr;
			// whether we have an active link
			bool linkUp = false;

			// unicast MAC address
			stack_mac_addr_t mac;

			// our IP address
			stack_ipv4_addr_t ip = 0x00000000;
			// network mask
			stack_ipv4_addr_t netMask = 0xFFFFFFFF;
			// router address
			stack_ipv4_addr_t routerIp = 0x00000000;

			// ethernet header and CRC
			static const size_t maxPayloadLength = net::EthMAC::MTU - 18 - 4;
	};
} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* STACK_H_ */
