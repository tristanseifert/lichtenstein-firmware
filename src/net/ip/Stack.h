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
	class IPv4;
	class DHCPClient;

	class UDPSocket;

	class Stack {
		public:
			Stack(Network *n);
			virtual ~Stack();

		// API for configuring the Ethernet and IP properties
		public:
			void setUnicastMACAddress(uint8_t *src);

			void linkStateChanged(bool linkUp);

			stack_ipv4_addr_t getIPAddress(void) const;
			bool ipAddressValid(void) const {
				return this->isIPv4ConfigValid;
			}

		// sockets
		public:
			UDPSocket *createUDPSocket(void);

		// API for receiving/transmitting packets
		public:
			void receivedPacket(void *data, size_t length, uint32_t userData);

			bool resolveIPToMAC(stack_ipv4_addr_t addr, stack_mac_addr_t *result, int timeout = 100);

		// helper APIs for MAC filtering
		public:
			void addMulticastMAC(stack_mac_addr_t addr);
			void removeMulticastMAC(stack_mac_addr_t addr);

		// private calls for packet handling
		private:
			void doneWithRxPacket(void *);


			void *getTxPacket(size_t, int timeout = -1);

			void discardTXPacket(void *);
			int sendTxPacket(void *, stack_mac_addr_t, uint16_t);
			inline int broadcastTxPacket(void *tx, uint16_t type) {
				return this->sendTxPacket(tx, kMACAddressBroadcast, type);
			}

		// protocol handlers
		private:
			friend class ARP;
			friend class IPv4;

			ARP *arp = nullptr;
			IPv4 *ipv4 = nullptr;

		// helpers
		public:
			static void ipToString(stack_ipv4_addr_t addr, char *out, size_t outLen);
			static void macToString(stack_mac_addr_t addr, char *out, size_t outLen);

			bool isIPLocal(stack_ipv4_addr_t addr);
			static bool isIPInSubnet(stack_ipv4_addr_t addr, stack_ipv4_addr_t netAddr, stack_ipv4_addr_t netmask);

			void setHostname(const char *hostname);
			const char *getHostname(void) const {
				return this->hostname;
			}

			const stack_mac_addr_t getMacAddress(void) const {
				return this->mac;
			}

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

			// hostname
			char *hostname = nullptr;

			// ethernet header and CRC
			static const size_t maxPayloadLength = net::EthMAC::MTU - 18 - 4;

		private:
			friend class DHCPClient;

			// only when set are IP packets handled
			bool isIPv4ConfigValid = false;

			// set to use DHCP to configure the network
			bool useDHCP = true;
			DHCPClient *dhcp = nullptr;

		private:
			void ipConfigBecameValid(void);
	};
} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* STACK_H_ */
