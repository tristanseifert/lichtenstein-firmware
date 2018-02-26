/*
 * EthPHY.h
 *
 * Exposes some common features for PHYs that the network driver may need to
 * use.
 *
 * @note This class should not be instantiated directly: instead, use the
 * factory method.
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */

#ifndef NET_ETHPHY_H_
#define NET_ETHPHY_H_

#include "EthPHYTypes.h"

#include <LichtensteinApp.h>

#include <cstdint>

class Network;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace net {
	class EthMAC;

	class EthPHY {
		public:
			EthPHY(Network *net, EthMAC *mac, bool rmii, uint16_t address);
			virtual ~EthPHY();

		public:
			static EthPHY *phyForId(Network *net, uint32_t id, uint16_t address, bool useRMII);

		public:
			/**
			 * Resets the PHY.
			 */
			virtual void reset(void) = 0;


			/**
			 * If the PHY is capable of loop-back mode, sets its state.
			 */
			virtual void setLoopbackState(bool enabled) = 0;


			/**
			 * Determines whether there is a link up on the PHY.
			 */
			virtual bool isLinkUp(void) = 0;
			/**
			 * Determines whether the PHY is operating in full duplex mode.
			 */
			virtual bool isFullDuplex(void) = 0;
			/**
			 * Returns the speed the PHY is operating at.
			 */
			virtual net_link_speed_t getSpeed(void) = 0;
			/**
			 * If the PHY is capable of detecting whether the network cable is
			 * a crossover (MDIX), return the detected state.
			 */
			virtual bool isMDIPairSwapped(void) = 0;


			/**
			 * Forces the PHY to perform link auto-negotiation.
			 */
			virtual void performAutonegotiation(void) = 0;
			/**
			 * If the autonegotation is not desired, set the link speed and
			 * duplex mode manually with this function.
			 *
			 * If these parameters could not be satisfied, false is returned.
			 */
			virtual bool setLinkParams(net_link_speed_t speed, bool duplex) = 0;


			/**
			 * Sets the PHY's power state. When the PHY is in power-down mode,
			 * only its registers may be accessed and Ethernet traffic WILL NOT
			 * be passed.
			 */
			virtual void setPowerState(bool powerUp) = 0;

		// link monitor
		protected:
			friend void EthPHYLinkMonitorTimerCallback(TimerHandle_t);

			void setUpLinkMonitor(void);
			void checkForLinkStateChange(void);

		private:
			bool lastLinkState = false;

			// timer used for the link monitor
			TimerHandle_t linkMon = nullptr;
			// how many ms should elapse between link checks
			static const int linkMonitorTimerInterval = 500;

		// locking
		protected:
			bool startMDIOTransaction(int timeout = -1);
			bool endMDIOTransaction(void);

			// this semaphore may be used to control access to the MDIO bus
			SemaphoreHandle_t mdioLock = nullptr;


		// MAC, network stack, and other parameters
		protected:
			Network *net = nullptr;
			EthMAC *mac = nullptr;

			// set if the MAC operates in RMII mode
			bool rmii = false;
			// PHY address
			uint16_t addr = 0;
	};

} /* namespace net */

#pragma GCC diagnostic pop

#endif /* NET_ETHPHY_H_ */
