/*
 * Network.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef NET_NETWORK_H_
#define NET_NETWORK_H_

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Network {
	public:
		static void init(void);
		static Network *sharedInstance(void) noexcept;

	private:
		Network();

		void startNetServices(void);

	private:
		void setUpMAC(void);
		void setUpPHY(void);

		void setUpStack();

	private:
		static const uint8_t ethParamMACOffset = 0xFA;

		uint8_t macAddress[6];

		void setUpEthParamEEPROM(void);
		void probeEthParamEEPROM(void);
		void writeEthParamEEPROM(void);

		void _i2cScan(void);

	private:
		virtual ~Network();
};

#pragma GCC diagnostic pop

#endif /* NET_NETWORK_H_ */
