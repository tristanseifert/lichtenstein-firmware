/*
 * Socket.h
 *
 * Abstract base class for sockets, providing a common least common denominator
 * API for sockets of every protocol. Each protocol can implement additional
 * functions and private calls used by the stack.
 *
 *  Created on: Mar 1, 2018
 *      Author: tristan
 */
#ifndef NET_IP_SOCKET_H_
#define NET_IP_SOCKET_H_

#include <cstddef>

#include "StackTypes.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace ip {
	class IPv4;
	class UDPSocket;

	class Socket {
		friend class UDPSocket;

		public:
			/**
			 * Socket options
			 */
			typedef enum {
				/**
				 * Are broadcast packets accepted?
				 *
				 * Type: bool
				 */
				kSockOptAcceptBroadcast = 1,

				/**
				 * Are multicast packets accepted?
				 *
				 * Type: bool
				 */
				kSockOptAcceptMulticast,

				/**
				 * Joins a multicast group.
				 *
				 * Type: stack_ipv4_addr_t
				 */
				kSockOptJoinMulticast,
				/**
				 * Leaves a multicast group.
				 *
				 * Type: stack_ipv4_addr_t
				 */
				kSockOptLeaveMulticast,
			} socket_option_t;

			/**
			 * Various protocols that can have their options set
			 */
			typedef enum {
				kSocketProtocolInternal,

				kSocketProtocolIPv4,
				kSocketProtocolUDP
			} socket_protocol_t;

			/**
			 * Socket errors
			 */
			enum {
				ErrSuccess					= 0,

				ErrTimeout					= -10000,
				ErrReceiveIO					= -10001,
				ErrMemory					= -10002,
				ErrUnknownBuffer				= -10003,
				ErrNotConnected				= -10004,
				ErrNoBookkeepingSpace		= -10005,
				ErrInvalidProtocol			= -10006,
				ErrInvalidOption				= -10007,
				ErrInvalidOptionLength		= -10008,
				ErrPortInUse					= -10009,

				ErrNotOpen					= -20000,
				ErrNotClosed					= -20001,

				ErrMulticastError			= -30000,

				ErrUnimplemented				= -99999,
			};

		public:
			virtual ~Socket();

		private:
			/**
			 * Private base constructor. Each subclass should define its own
			 * constructor.
			 */
			Socket();

		public:
			/**
			 * Blocks until a packet is received.
			 *
			 * @param buffer Address of a void * pointer which will hold the
			 * memory address of the receive buffer when a packet is received,
			 * or `nullptr` if an error occurred.
			 * @param bytesRead Address of a `size_t` variable that will
			 * contain the total number of bytes of user payload in the packet
			 * that was received, or UINT_MAX if an error occurred.
			 * @param timeout How long to wait for a packet to be received, in
			 * OS ticks. Negative values will block forever.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int receive(void **buffer, size_t *bytesRead, unsigned int timeout = -1) = 0;

			/**
			 * Indicates to the stack that the application has finished
			 * processing a packet returned by a previous call to `receive,`
			 * and does not wish to use the buffer any more. This returns the
			 * buffer to the pool of available receive descriptors.
			 *
			 * @note This function should be called on every buffer that was
			 * received, even if the buffer wasn't used.
			 *
			 * @param buffer Address of a buffer previously returned by a call
			 * to `receive`.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int discardRx(void *buffer) = 0;


		public:
			/**
			 * Gets a transmit buffer for the application to write data in to,
			 * containing the specified number of bytes.
			 *
			 * @param buffer Address of a void * pointer which will hold the
			 * memory address of the transmit buffe, or `nullptr` if an error
			 * occurred.
			 * @param length How many bytes of payload the application wants to
			 * send
			 * @param timeout How long to wait for a transmit buffer to become
			 * available, in OS ticks. Negative values will block forever.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int prepareTx(void **buffer, size_t length, unsigned int timeout = -1) = 0;

			/**
			 * Transmits a buffer that was retrieved with an earlier call to
			 * `prepareTx.`
			 *
			 * @param buffer Address of the buffer retrieved by a call to
			 * `prepareTx`
			 * @param timeout How long to wait for a transmit buffer to be
			 * queued for transmission, in OS ticks. Negative values will
			 * block forever.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int queueTx(void *buffer, unsigned int timeout = -1) = 0;

			/**
			 * Discards a buffer retrieved by a call to `prepareTx` without
			 * actually sending it.
			 *
			 * @param buffer Address of the buffer retrieved by a call to
			 * `prepareTx`
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int discardTx(void *buffer) = 0;


		public:
			/**
			 * Sets up a socket for communication.
			 *
			 * @note This must be called before any of the other functions on
			 * this class are called.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int open(void) = 0;

			/**
			 * Closes the socket, waiting for any pending transmit requests to
			 * be completed. This function will block until that is done.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int close(void) = 0;

			/**
			 * Establishes a connection to a remote address and port.
			 *
			 * For connection-oriented protocols such as TCP, this will perform
			 * the required handshaking and sets up a full-duplex connection.
			 *
			 * In the case of connectionless protocols like UDP, this just
			 * sets the destination to which any later packets are sent.
			 *
			 * @param address Remote IP address
			 * @param port Remote port
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int connect(stack_ipv4_addr_t address, unsigned int port) = 0;

			/**
			 * Sets up the socket to accept packets (i.e. entering listening
			 * mode) on a particular port.
			 *
			 * @return 0 if successful, an error code otherwise.
			 */
			virtual int bind(unsigned int port) = 0;


		public:
			/**
			 * Sets a particular socket option.
			 *
			 * @note Not all sockets support all operations. For example, a
			 * socket using IPv4 as its transport will not support some of the
			 * options that an IPv6 socket may, and vice-versa.
			 *
			 * @param protocol Which protocol to set an option for: this could
			 * be an underlying transport (IPv4) or any of the other protocols
			 * (such as UDP or TCP) that run on that transport.
			 * @param option The option to set
			 * @param value Pointer to the value to set the option to. This is
			 * option-dependant.
			 * @param length Length of the value, in bytes.
			 *
			 * Returns 0 if the option was set, an error code otherwise.
			 */
			virtual int setSockOpt(socket_protocol_t protocol, socket_option_t option, void *value, size_t length) = 0;

			/**
			 * Gets a particular socket option.
			 *
			 * @note Not all sockets support all operations. For example, a
			 * socket using IPv4 as its transport will not support some of the
			 * options that an IPv6 socket may, and vice-versa.
			 *
			 * @param protocol Which protocol to get an option for: this could
			 * be an underlying transport (IPv4) or any of the other protocols
			 * (such as UDP or TCP) that run on that transport.
			 * @param option The option to get
			 * @param value Pointer to the value to write the option's current
			 * value to.
			 * @param length Length of the value, in bytes.
			 *
			 * Returns 0 if the option was set, an error code otherwise.
			 */
			virtual int getSockOpt(socket_protocol_t protocol, socket_option_t option, void *out, size_t length) = 0;


		protected:
			/**
			 * Pointer to the IPv4 handler (used to set multicast stuff)
			 */
			IPv4 *ipv4 = nullptr;

		protected:
			/**
			 * Indicates whether the socket has been opened.
			 */
			bool isOpened = false;
			/**
			 * Indicates whether the socket has been connected.
			 */
			bool isConnected = false;

			/**
			 * Remote address
			 */
			stack_ipv4_addr_t remoteAddr = kIPv4AddressZero;
			/**
			 * Remote port
			 */
			unsigned int remotePort = 0;
			/**
			 * Local port: this is only set (i.e. non-zero) if a call to `bind`
			 * was previously made.
			 */
			unsigned int localPort = 0;
	};
} /* namespace ip */

#pragma GCC diagnostic pop

#endif /* NET_IP_SOCKET_H_ */
