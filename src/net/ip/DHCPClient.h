/*
 * DHCPClient.h
 *
 * Implements a DHCP client.
 *
 *  Created on: Mar 4, 2018
 *      Author: tristan
 */

#ifndef NET_IP_DHCPCLIENT_H_
#define NET_IP_DHCPCLIENT_H_

#include <LichtensteinApp.h>

#include "StackTypes.h"

namespace ip {
	class Stack;
	class UDPSocket;

	class DHCPClient {
		public:
			DHCPClient(Stack *stack);
			virtual ~DHCPClient();

		public:
			void requestIP(void);
			void reset(void);

		// byte order conversion helpers
		private:
			void convertPacketByteOrder(void *);

			void packetNetworkToHost(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}
			void packetHostToNetwork(void *_packet) {
				this->convertPacketByteOrder(_packet);
			}

		private:
			// DHCP task priority
			static const int taskPriority = 1;
			// task stack size
			static const size_t taskStackSize = 150;

			// timeout for receiving DHCP messages
			static const TickType_t receiveTimeout = (5000 / portTICK_PERIOD_MS);

			// UDP socket used by the task
			UDPSocket *sock = nullptr;

			// task
			TaskHandle_t task = nullptr;

			// this mutex is given every time the state changes
			SemaphoreHandle_t stateChangeMutex = nullptr;

			Stack *stack = nullptr;

		private:
			enum {
				// waiting for a task
				IDLE,

				// send a DHCPDISCOVER
				DISCOVER,
				// waiting for an offer
				WAITOFFER,
				// send a DHCPREQUEST
				REQUEST,
				// waiting for acknowledgement
				WAITACK,
				// set the IP configuration
				SUCCESS,

				// renew the existing lease
				RENEW,
				// wait for a DHCPACK for the renew
				WAITRENEWACK,

				// if any of the waits time out, go here
				TIMEOUT
			};

			// state of the DHCP state machine
			unsigned int state = IDLE;

			// transaction ID we sent with the discover
			uint32_t currentXID = 0;

			// last offer
			struct {
				// address the DHCP server offered us
				stack_ipv4_addr_t address;
				// subnet mask the DHCP server offered us
				stack_ipv4_addr_t netmask;

				// router address
				stack_ipv4_addr_t router;
				// DNS server address
				stack_ipv4_addr_t dnsServer;
				// log server address
				stack_ipv4_addr_t syslogServer;
				// TFTP server address
				stack_ipv4_addr_t tftpServer;

				// NTP server address
				stack_ipv4_addr_t ntpServer;
				// offset from UTC, in seconds
				unsigned int utcOffset;

				// how long the lease is good for, in seconds
				unsigned int leaseExpiration;

				// IP address of the DHCP server
				stack_ipv4_addr_t serverAddress;
			} offer;

			// changes state and notifies task
			inline void changeState(unsigned int newState) {
				this->state = newState;
				xSemaphoreGive(this->stateChangeMutex);
			}

		private:
			// renewal timer: expires after 1/2 of the lease duration
			TimerHandle_t renewalTimer = nullptr;

		private:
			int parseOptions(void *);

			void printOfferInfo(void);

		private:
			friend void _DHCPRenewTimeout(TimerHandle_t);
			friend void _DHCPClientTaskTrampoline(void *);

			int taskEntry(void);

			void taskSendDiscover(void);
			void taskHandleOffer(void);
			void taskSendRequest(void);
			void taskHandleAck(void);
			void taskUpdateIPConfig(void);

			void taskRenewLease(void);
			void taskRenewWaitAck(void);

			void setUpRenewalTimer(void);

			void fillRequestOptions(void *, size_t);
	};

} /* namespace ip */

#endif /* NET_IP_DHCPCLIENT_H_ */
