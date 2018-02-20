/*
 * DP83848C.h
 *
 * Implements a PHY interface for the DP83848C Ethernet transceiver.
 *
 * Datasheet: http://www.ti.com/lit/ds/snls266e/snls266e.pdf
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */

#ifndef NET_PHY_DP83848C_H_
#define NET_PHY_DP83848C_H_

#include "../EthPHY.h"

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace net {
	class DP83848C: public EthPHY {
		public:
			DP83848C(Network *net, EthMAC *mac, bool rmii, uint16_t address);
			virtual ~DP83848C();

		// functions inherited from EthPHY
		public:
			void reset(void);

			void setLoopbackState(bool enabled);

			bool isLinkUp(void);
			bool isFullDuplex(void);
			net_link_speed_t getSpeed(void);
			bool isMDIPairSwapped(void);

			void performAutonegotiation(void);
			bool setLinkParams(net_link_speed_t speed, bool duplex);

			void setPowerState(bool powerUp);

		private:
			void setUpRegisters(void);
			bool runBIST(void);

			uint16_t readStatus(void);
	};

} /* namespace net */

#pragma GCC diagnostic pop

#endif /* NET_PHY_DP83848C_H_ */
