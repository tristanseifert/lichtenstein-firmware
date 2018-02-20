/*
 * DP83848CPrivate.h
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */

#ifndef NET_PHY_DP83848CPRIVATE_H_
#define NET_PHY_DP83848CPRIVATE_H_

/**
 * Basic Mode Control Register
 *
 * - bit 15: Reset. Write 1 to reset, wait until 0 for reset complete.
 */
#define	MDIO_REG_BMCR						0x00

// write this bit as 1 to reset, poll for it to return to 0 for completion
#define MDIO_REG_BMCR_RESET					(1 << 15)
// when set, transmitted data is looped back to the MII RX outputs.
#define MDIO_REG_BMCR_LOOPBACK				(1 << 14)
// if autonegotiation is disabled, use 100Mbps if set.
#define MDIO_REG_BMCR_100Mbps				(1 << 13)
// enables autonegotiation if set
#define MDIO_REG_BMCR_AUTONEG				(1 << 12)
// places the PHY into a power-down state when set
#define MDIO_REG_BMCR_POWERDOWN				(1 << 11)
// isolates the Ethernet port from the PHY
#define MDIO_REG_BMCR_ISOLATE				(1 << 10)
// restart autonegotiation
#define MDIO_REG_BMCR_RESTART_AUTONEG		(1 << 9)
// if autonegotiation is disabled, operate in full duplex if set.
#define MDIO_REG_BMCR_FULL_DUPLEX			(1 << 8)
// when set, the collision test is enabled.
#define MDIO_REG_BMCR_COL_TEST				(1 << 7)


/**
 * Basic mode status register
 */
#define MDIO_REG_BMSR						0x01

// when set, the device is capable of operating in 100BASE-TX FD
#define MDIO_REG_BMSR_100BASETX_FD			(1 << 14)
// when set, the device is capable of operating in 100BASE-TX HD
#define MDIO_REG_BMSR_100BASETX_HD			(1 << 13)

// when set, the device is capable of operating in 10BASE-T FD
#define MDIO_REG_BMSR_10BASET_FD				(1 << 12)
// when set, the device is capable of operating in 10BASE-T HD
#define MDIO_REG_BMSR_10BASET_HD				(1 << 11)

// the autonegotiation process is complete when set
#define MDIO_REG_BMSR_AUTONEG_COMPLETE		(1 << 5)

// remote station has indicated a fault
#define MDIO_REG_BMSR_REMOTE_FAULT			(1 << 4)

// the remote device supports autonegotiation
#define MDIO_REG_BMSR_AUTONEG_ABILITY		(1 << 3)

// indicates that a valid link is established
#define MDIO_REG_BMSR_LINK_UP				(1 << 2)

// indicates that jabber has been detected (only in 10BASE-T)
#define MDIO_REG_BMSR_JABBER					(1 << 1)

// extended capabilities available
#define MDIO_REG_BMSR_EXTENDED				(1 << 0)


// TODO: add more registers

/**
 * PHY Control Register
 */
#define MDIO_REG_PHYCR						0x19

// auto-MDIX capability
#define MDIO_REG_PHYCR_AUTO_MDIX				(1 << 15)
// MAC can receive pause frames
#define MDIO_REG_PHYCR_PAUSE_RX				(1 << 13)
// MAC can transmit pause frames
#define MDIO_REG_PHYCR_PAUSE_TX				(1 << 12)

// use the longer BIST sequence when set
#define MDIO_REG_PHYCR_PSR_15				(1 << 10)
// set if the BIST passes.
#define MDIO_REG_PHYCR_BIST_PASS				(1 << 9)
// set to begin the BIST
#define MDIO_REG_PHYCR_BIST_START			(1 << 8)

// disable LED stretching
#define MDIO_REG_PHYCR_DISABLE_LED_STRETCH	(1 << 7)
// LED config bit 1
#define MDIO_REG_PHYCR_LED_CFG1				(1 << 6)
// LED config bit 0
#define MDIO_REG_PHYCR_LED_CFG0				(1 << 5)
// LED config mask
#define MDIO_REG_PHYCR_LED_CFG				(MDIO_REG_PHYCR_LED_CFG0 | MDIO_REG_PHYCR_LED_CFG1)

// PHY address mask
#define MDIO_REG_PHYCR_PHY_ADDR				0x001F


#endif /* NET_PHY_DP83848CPRIVATE_H_ */
