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


/**
 * Auto-Negotiation Advertisement Register
 */
#define MDIO_REG_ANAR						0x04

// next page transfer is desired
#define MDIO_REG_ANAR_NP_DESIRED				(1 << 15)
// set when a remote fault has been detected
#define MDIO_REG_ANAR_REMOTE_FAULT			(1 << 14)

// support for asymmetric PAUSE frames
#define MDIO_REG_ANAR_ASYM_PAUSE				(1 << 11)
// support for PAUSE frames on full duplex links
#define MDIO_REG_ANAR_PAUSE					(1 << 10)

// PHY can support 100BASE-TX full duplex operation
#define MDIO_REG_ANAR_100BASE_TX_FD			(1 << 8)
// PHY can support 100BASE-TX half duplex operation
#define MDIO_REG_ANAR_100BASE_TX_HD			(1 << 7)
// PHY can support 10BASE-T full duplex operation
#define MDIO_REG_ANAR_10BASE_T_FD			(1 << 6)
// PHY can support 10BASE-T half duplex operation
#define MDIO_REG_ANAR_10BASE_T_HD			(1 << 5)

// mask for protocol selector
#define MDIO_REG_ANAR_SELECTOR_MASK			(0x1F)
// default protocol selector value for Ethernet
#define MDIO_REG_ANAR_SELECTOR_ETH			(0b0001)

/**
 * Auto-Negotiation Link Partner Ability Register
 *
 * Use the same defines as for the ANAR register.
 */
#define MDIO_REG_ANLPAR						0x05
/**
 * Auto-Negotiation Link Partner Ability Register (next page)
 *
 * Use the same defines as for the ANNPTR register.
 */
#define MDIO_REG_ANLPAR						0x05

/**
 * Auto-Negotiation Expansion Register
 */
#define MDIO_REG_ANER						0x06

// parallel detection fault
#define MDIO_REG_ANER_PDF					(1 << 4)
// link partner can send Next Page
#define MDIO_REG_ANER_PARTNER_NP_ABLE		(1 << 3)
// do we support sending the next page?
#define MDIO_REG_ANER_NP_ABLE				(1 << 2)
// has a new link code word been received?
#define MDIO_REG_ANER_PAGE_RX				(1 << 1)
// is the link partner capable of autonegotiation?
#define MDIO_REG_ANER_AUTONEG_ABLE			(1 << 0)

/**
 * Auto-Negotiation Next Page Transmit Register
 */
#define MDIO_REG_ANNPTR						0x07

// is another page desired?
#define MDIO_REG_ANNPTR_NP					(1 << 15)
// is this a message page?
#define MDIO_REG_ANNPTR_MP					(1 << 13)

// acknowledge bit: set that the message can be complied with
#define MDIO_REG_ANNPTR_ACK2					(1 << 12)
// toggle bit (inverse of the toggle bit in the previously sent word
#define MDIO_REG_ANNPTR_TOGGLE				(1 << 11)

// code (content of the frame)
#define MDIO_REG_ANNPTR_CODE_MASK			(0b11111111111)


/**
 * PHY Status Register
 */
#define MDIO_REG_PHYSTS						0x10

// current MDIX mode; set if MDI pairs are swapped
#define MDIO_REG_PHYSTS_MDI_SWAPPED			(1 << 14)

// receive error latch (set if a receive error occurred since reading RXERCNT)
#define MDIO_REG_PHYSTS_RX_ERR_LATCH			(1 << 13)
// polarity detection: set if the polarity is reversed
#define MDIO_REG_PHYSTS_POLARITY_REVERSED	(1 << 12)
// false carrier sense latch (set if a false carrier was sensed since reading FCSR)
#define MDIO_REG_PHYSTS_FALSE_CCARRIER		(1 << 11)

// 100BASE-TX qualified signal detect
#define MDIO_REG_PHYSTS_100BASE_TX_SIGNAL	(1 << 10)
// 100BASE-TX scrambler lock
#define MDIO_REG_PHYSTS_SCRAMBLER_LOCK		(1 << 9)

// was a link code word page received? (cleared reading ANER)
#define MDIO_REG_PHYSTS_PAGE_RX				(1 << 8)

// is an MII interrupt pending?
#define MDIO_REG_PHYSTS_MII_IRQ_PENDING		(1 << 7)

// was a remote fault detected?
#define MDIO_REG_PHYSTS_REMOTE_FAULT			(1 << 6)
// was jabber detected (valid only in 10BASE-T mode)
#define MDIO_REG_PHYSTS_JABBER				(1 << 5)

// is autonegotiation complete?
#define MDIO_REG_PHYSTS_AUTONEG_COMPLETE		(1 << 4)

// is loopback enabled?
#define MDIO_REG_PHYSTS_LOOPBACK				(1 << 3)

// is the link full duplex?
#define MDIO_REG_PHYSTS_FULL_DUPLEX			(1 << 2)
// is the link operating at 10mbps?
#define MDIO_REG_PHYSTS_10MBPS				(1 << 1)
// is a valid link established?
#define MDIO_REG_PHYSTS_LINK_VALID			(1 << 0)


/**
 * False carrier detect counter
 *
 * This is an 8 bit counter and it saturates at 0xFF. The value is reset to
 * zero on read.
 */
#define MDIO_REG_FCSCR						0x14

/**
 * RX error counter
 *
 * This is an 8 bit counter and it saturates at 0xFF. The value is reset to
 * zero on read.
 */
#define MDIO_REG_RECR						0x15


/**
 * 100Mbps Physical Coding Sublayer Configuration
 */
#define MDIO_REG_PCSR						0x16

// receive clock???
#define MDIO_REG_PCSR_FREE_CLK				(1 << 11)

// enable transmit true quiet mode (idk, this should be off)
#define MDIO_REG_PCSR_TRUE_QUIET				(1 << 10)
// force signal detection if set
#define MDIO_REG_PCSR_FORCE_PMA				(1 << 9)


// set to not drop link status on descrambler lock
#define MDIO_REG_PCSR_SCRAMBLER_LOCK_LOSS	(1 << 8)
// set descrambler lock timeout to 2ms (vs 722µS) when set
#define MDIO_REG_PCSR_SCRAMBLER_LOCK_TIMEOUT	(1 << 7)

// force a 100Mbps good link
#define MDIO_REG_PCSR_FORCE_100_OK			(1 << 5)

// bypass the NRZI encoding
#define MDIO_REG_PCSR_BYPASS_NRZI			(1 << 2)


/**
 * MII interface configuration
 *
 * Receive elasticity defines the tolerance in the RMII clock vs. the received
 * data, in terms of bits:
 *
 * - 00: 14 bit tolerance (up to 16,800 byte packets)
 * - 01: 2 bit tolerance (up to 2,400 byte packets)
 * - 10: 6 bit tolerance (up to 7,200 byte packets)
 * - 11: 10 bit tolerance (up to 12,000 byte packets)
 */
#define MDIO_REG_RBR							0x17

// use the reduced MII (RMII) interface
#define MDIO_REG_RBR_RMII					(1 << 5)
// use RMII 1.0 (1.2 is used if cleared)
#define MDIO_REG_RBR_RMII_1_0				(1 << 4)

// RX FIFO overflow status
#define MDIO_REG_RBR_RX_OVERFLOW				(1 << 3)
// RX FIFO underflow status
#define MDIO_REG_RBR_RX_UNDERFLOW			(1 << 2)

// receive elasticity buffer mask
#define MDIO_REG_RBR_ELAST_BUF_MASK			(0x03)


/**
 * LED Direct Control Register
 */
#define MDIO_REG_LEDCR						0x18

// when set, the speed LED is manually controlled
#define MDIO_REG_LEDCR_SPD_MODE				(1 << 5)
// when set, the link LED is manually controlled
#define MDIO_REG_LEDCR_LINK_MODE				(1 << 4)
// when set, the activity LED is manually controlled
#define MDIO_REG_LEDCR_ACT_MODE				(1 << 3)

// state of the speed LED
#define MDIO_REG_LEDCR_SPD					(1 << 2)
// state of the link LED
#define MDIO_REG_LEDCR_LINK					(1 << 1)
// state of the activity LED
#define MDIO_REG_LEDCR_ACT					(1 << 0)


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


/**
 * 10BASE-T Control Register
 */
#define MDIO_REG_10BTSCR						0x1A

// Use the serial network interface transmit mode
#define MDIO_REG_10BTSCR_SERIAL				(1 << 15)

// Squelch threshold (defaults to 330mV peak to peak)
#define MDIO_REG_10BTSCR_SQUELCH_MASK		(0xE00)
// default squelch value of 330mV peak to peak
#define MDIO_REG_10BTSCR_SQUELCH_330MV		(0x800)

// disable loopback of transmitted packets in half-duplex mode
#define MDIO_REG_10BTSCR_LOOPBACK_DIS		(1 << 8)

// disable transmission of the normal link pulse
#define MDIO_REG_10BTSCR_LP_DIS				(1 << 7)

// force a good 10Mbps link
#define MDIO_REG_10BTSCR_FORCE_LINK			(1 << 6)
// is the polarity inversed?
#define MDIO_REG_10BTSCR_POLARITY_INVERSE	(1 << 4)

// disable the heartbeat function
#define MDIO_REG_10BTSCR_HEARTBEAD_DIS		(1 << 1)
// disable jabber function
#define MDIO_REG_10BTSCR_JABBER_DIS			(1 << 0)

#endif /* NET_PHY_DP83848CPRIVATE_H_ */
