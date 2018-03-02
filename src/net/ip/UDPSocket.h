/*
 * UDPSocket.h
 *
 *  Created on: Mar 2, 2018
 *      Author: tristan
 */

#ifndef NET_IP_UDPSOCKET_H_
#define NET_IP_UDPSOCKET_H_

#include "Socket.h"

namespace ip {
	class UDP;

	class UDPSocket: public Socket {
		public:
			UDPSocket(UDP *_udp);
			virtual ~UDPSocket();

		public:
			virtual int receive(void **buffer, size_t *bytesRead, int timeout = -1);
			virtual int discardRx(void *buffer);

		public:
			virtual int prepareTx(void **buffer, size_t length, int timeout = -1);
			virtual int queueTx(void *buffer, int timeout = -1);
			virtual int discardTx(void *buffer);


		public:
			virtual int open(void) = 0;
			virtual int close(void) = 0;

			virtual int connect(stack_ipv4_addr_t address, unsigned int port);
			virtual int bind(unsigned int port);


		public:
			virtual int setSockOpt(socket_protocol_t protocol, socket_option_t option, void *value, size_t length);
			virtual int getSockOpt(socket_protocol_t protocol, socket_option_t option, void *out, size_t length);

		private:
			UDP *udp = nullptr;
	};

} /* namespace ip */

#endif /* NET_IP_UDPSOCKET_H_ */
