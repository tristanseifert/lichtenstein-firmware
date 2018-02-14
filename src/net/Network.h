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
		static const uint8_t ethParamI2CAddress = 0x50;
		static const uint8_t ethParamMACOffset = 0x80;

		uint8_t macAddress[6];

		void setUpEthParamEEPROM(void);
		void probeEthParamEEPROM(void);

		void _i2cScan(void);

		void i2cWaitForIdle(void);
		int i2cStart(uint8_t address, bool read = true, int timeout = 50000);
		void i2cStop(void);
		void i2cWriteByte(uint8_t data);
		uint8_t i2cReadByte(bool ack = true);

	private:
		virtual ~Network();
};

#pragma GCC diagnostic pop

#endif /* NET_NETWORK_H_ */
