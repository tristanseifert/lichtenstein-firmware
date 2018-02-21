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

namespace net {
	class EthMAC;
	class EthPHY;
}

class Network {
	friend class net::EthMAC;
	friend class net::EthPHY;

	public:
		static void init(void);
		static Network *sharedInstance(void) noexcept;

	private:
		Network();

		void startNetServices(void);

	private:
		void setUpClocks(void);
		void setUpMAC(void);
		void setUpEthernetGPIOs(void);

		void scanForPHYs(void);

		void setUpStack();

	private:
		net::EthMAC *mac = nullptr;
		net::EthPHY *phy = nullptr;

	// callbacks from PHY
	protected:
		void _phyLinkStateChange(bool isLinkUp);

	// MAC address reading
	private:
		static const uint8_t ethParamMACOffset = 0xFA;

		uint8_t macAddress[6];

		void readMACFromEEPROM(void);

	private:
		virtual ~Network();
};

#pragma GCC diagnostic pop

#endif /* NET_NETWORK_H_ */
