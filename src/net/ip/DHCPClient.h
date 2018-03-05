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

namespace ip {
	class Stack;

	class DHCPClient {
		public:
			DHCPClient(Stack *stack);
			virtual ~DHCPClient();

		public:
			void requestIP(void);
			void reset(void);

		private:
			// DHCP task priority
			static const int taskPriority = 1;
			// task stack size
			static const size_t taskStackSize = 100;

			// timeout for receiving DHCP messages
			static const TickType_t receiveTimeout = (5000 / portTICK_PERIOD_MS);

			// task
			TaskHandle_t task = nullptr;
			// timeout timer
			TimerHandle_t timeoutTimer = nullptr;

			// this handle is given every time the state changes
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

				// if any of the waits time out, go here
				TIMEOUT
			};

			// state of the DHCP state machine
			unsigned int state = IDLE;

			// changes state and notifies task
			inline void changeState(unsigned int newState) {
				this->state = newState;
				xSemaphoreGive(this->stateChangeMutex);
			}

		private:
			friend void _DHCPClientTaskTrampoline(void *);

			int taskEntry(void);
	};

} /* namespace ip */

#endif /* NET_IP_DHCPCLIENT_H_ */
