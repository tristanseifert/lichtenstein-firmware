/*
 * EthMAC.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */
#define LOG_MODULE "MAC"

#include "EthMAC.h"
#include "Network.h"

#include <LichtensteinApp.h>

namespace net {

/**
 * Initializes the MAC.
 */
EthMAC::EthMAC(Network *_net, bool useRMII) : net(_net), rmii(useRMII) {
	this->setUpClocks();

	this->setUpMACRegisters();
	this->setUpMMCRegisters();
}

/**
 * Sets default values in the MAC Ethernet-related registers. The default
 * values are:
 *
 * - 96 bits inter-frame gap, enable full duplex 100Mbps, IP checksum offload
 * - pass all multicast packets but filter unicast
 * - auto-generate pause frames
 * - disable VLAN tagging
 * - any unicast packets will generate power management events
 * - mask time stamp trigger interrupts
 */
void EthMAC::setUpMACRegisters(void) {

	// configure the MAC
	uint32_t cfg = ETH->MACCR;
	cfg &= 0xFF308103; // keep reserved bits

	cfg |= ETH_MACCR_IFG_96Bit; // 96 bit inter-frame gap
	cfg |= ETH_MACCR_FES; // enable fast ethernet
	cfg |= ETH_MACCR_ROD; // don't receive own frames in half-duplex
	cfg |= ETH_MACCR_DM; // enable full duplex
	cfg |= ETH_MACCR_IPCO; // enable IP checksum offloading
	cfg |= ETH_MACCR_BL_4; // back-off timer between 0 and 4
	cfg |= ETH_MACCR_DC; // wait 24,288 bits before giving up in half duplex

	cfg |= ETH_MACCR_TE; // enable transmitter
	cfg |= ETH_MACCR_RE; // enable receiver

	// initialize the frame filter
	uint32_t frameFilter = ETH->MACFFR;
	frameFilter &= 0x7FFFF800; // keep reserved bits

	frameFilter |= ETH_MACFFR_HPF; // enable hash/perfect filtering
	frameFilter |= ETH_MACFFR_PAM; // pass all multicast packets

	ETH->MACFFR = frameFilter;

	// clear the MAC hash table
	ETH->MACHTHR = 0;
	ETH->MACHTLR = 0;

	// configure flow control
	uint32_t flow = ETH->MACFCR;
	flow &= 0x0000FF00; // keep reserved bits

	flow |= (0x1000 << 16); // set pause time (1 = 512 bit times)
	flow |= ETH_MACFCR_PLT_Minus144; // re-transmit pause frame 144 slots later

	flow |= ETH_MACFCR_UPFD; // respond to pause frames with our unicast MAC

	flow |= ETH_MACFCR_RFCE; // act on receive pause frames (pauses tx)
	flow |= ETH_MACFCR_TFCE; // enable sending pause frames

	ETH->MACFCR = flow;

	// disable VLAN tagging
	uint32_t vlan = ETH->MACVLANTR;
	vlan &= 0xFFFE0000; // keep reserved bits

	ETH->MACVLANTR = vlan;

	// allow any received unicast frame to wake the processor
	uint32_t wakeup = ETH->MACPMTCSR;
	wakeup &= 0x7FFFFD98; // keep reserved bits

	wakeup |= ETH_MACPMTCSR_GU;

	ETH->MACPMTCSR = wakeup;

	// mask interrupts from the time stamp trigger
	uint16_t irqMask = ETH->MACIMR;
	irqMask &= 0xFDF7; // keep reserved bits

	irqMask |= ETH_MACIMR_TSTIM; // mask time stamp trigger interrupt

	ETH->MACIMR = irqMask;
}



/**
 * Sets up the clocks for the Ethernet peripheral.
 */
void EthMAC::setUpClocks(void) {
	// enable clock for the  DMA engines
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

	// enable clocks for the ETH peripherals
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	// reset the MAC
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
}

/**
 * De-initializes the MAC by turning off its clocks.
 */
EthMAC::~EthMAC() {
	// turn off clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, DISABLE);

	// reset the MAC
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
}



/**
 * Sets up the configuration of the MMC (management counters) registers.
 */
void EthMAC::setUpMMCRegisters(void) {
	uint32_t temp;

	// reset MMC counters
	temp = ETH->MMCCR;
	temp &= 0xFFFFFFF0; // keep reserved bits

	temp |= ETH_MMCCR_CR; // reset counters

	ETH->MMCCR = temp;

	// counters are self-clearing on read
	temp &= ~ETH_MMCCR_CR; // clear reset counters bit
	temp |= ETH_MMCCR_ROR; // counters will reset on read

	ETH->MMCCR = temp;

	// generate MMC IRQ when certain receive counters reach half full
	temp = ETH->MMCRIR;
	temp &= 0xFFFDFF9F; // keep reserved bits

	temp |= ETH_MMCRIR_RGUFS; // good unicast frames
	temp |= ETH_MMCRIR_RFAES; // frames with alignment errors
	temp |= ETH_MMCRIR_RFCES; // frames with CRC error

	ETH->MMCRIR = temp;

	// generate MMC IRQ when certain transmit counters reach half full
	temp = ETH->MMCTIR;
	temp &= 0xFFDF3FFF; // keep reserved bits

	temp |= ETH_MMCTIR_TGFS; // good frames
	temp |= ETH_MMCTIR_TGFMSCS; // good frames after a 2+ collisions
	temp |= ETH_MMCTIR_TGFSCS; // good frames after a single collision

	ETH->MMCTIR = temp;

	// do not mask any MMC IRQs for receive counter overflow
	temp = ETH->MMCRIMR;
	temp &= 0xFFFDFF9F; // keep reserved bits

	ETH->MMCRIMR = temp;

	// do not mask any MMC IRQs for transmit counter overflow
	temp = ETH->MMCTIMR;
	temp &= 0xFFDF3FFF; // keep reserved bits

	ETH->MMCTIMR = temp;
}

/**
 * Reads the values from all MMC counters. Since the counters are reset on
 * read, we just add their contents to what we've internally been keeping
 * track of.
 */
void EthMAC::readMMCCounters(void) {
	this->mmcTxAfterSingleCollision += ETH->MMCTGFSCCR;
	this->mmcTxAfterMultipleCollision += ETH->MMCTGFMSCCR;
	this->mmcTxFrames += ETH->MMCTGFCR;

	this->mmcRxBadCRC += ETH->MMCRFCECR;
	this->mmcRxAlignmentError += ETH->MMCRFAECR;
	this->mmcRxUnicast += ETH->MMCRGUFCR;
}



/**
 * Waits for the MDIO interface to no longer be busy by polling on the BUSY
 * bit in the MACMIIAR register.
 *
 * If the access times out, -1 is returned. 0 otherwise.
 */
int EthMAC::mdioWait(unsigned int timeout) {
	// wait for the bit to be reset
	while((ETH->MACMIIAR & ETH_MACMIIAR_MB) != 0) {
		// if it's set, decrement the timeout
		if(timeout-- == 0) {
			// if it's expired, return.
			return -1;
		}
	}

	// otherwise, assume success
	return 0;
}

/**
 * Formats an MDIO address and writes it into the appropriate register.
 */
int EthMAC::mdioSendAddress(uint16_t phy, uint16_t reg, bool write) {
	uint32_t temp;

	// wait for the MDIO interface to be ready
	// TODO: is this check required? i think not

	// read the old register to keep the reserved bits
	temp = ETH->MACMIIAR;
	temp &= 0xFFFF0020; // keep reserved bits

	temp |= ((phy & 0x1F) << 11) & ETH_MACMIIAR_PA; // PHY address
	temp |= ((reg & 0x1F) << 6) & ETH_MACMIIAR_MR; // register address

	temp |= ETH_MACMIIAR_CR_Div42; // MDIO clock is HCLK/42

	if(write) {
		temp |= ETH_MACMIIAR_MW; // access is a write
	} else {
		temp &= ~(ETH_MACMIIAR_MW); // access is a read
	}

	temp |= ETH_MACMIIAR_MB; // set to indicate an MDIO operation

	// write register
	ETH->MACMIIAR = temp;

	return 0;
}

/**
 * Reads from an MDIO register.
 *
 * @return A negative value if the read failed, the register value otherwise.
 */
int EthMAC::mdioRead(uint16_t phy, uint16_t reg) {
	// send address
	if(this->mdioSendAddress(phy, reg, false) != 0) {
		LOG(S_ERROR, "Timeout on address phy %d, reg %d", phy, reg);

		return -1;
	}

	// wait for the MDIO interface to not be busy anymore
	if(this->mdioWait() != 0) {
		LOG(S_ERROR, "Timeout waiting for MDIO ready");
		return -1;
	}

	// return the read data
	return (uint16_t) (ETH->MACMIIDR & ETH_MACMIIDR_MD);
}

/**
 * Performs a write to an MDIO register.
 */
int EthMAC::mdioWrite(uint16_t phy, uint16_t reg, uint16_t value) {
	int err;

	// set the data register first
	ETH->MACMIIDR = value;

	// send the address for a write
	if(this->mdioSendAddress(phy, reg, true) != 0) {
		LOG(S_ERROR, "Timeout on address phy %d, reg %d", phy, reg);

		return -1;
	}

	// wait for the MDIO interface to not be busy anymore
	err = this->mdioWait();

	if(err != 0) {
		LOG(S_ERROR, "Timeout waiting for MDIO ready");
	}

	return err;
}



/**
 * Writes a MAC address into a specific "slot" on the MAC and sets the enable
 * bit as described.
 *
 * Note that the SA bit is cleared so that the MAC is compared against the
 * destination MAC address of an Ethernet frame, and all bits in the MBC
 * field are cleared so the MAC performs a comparison against all six bytes.
 *
 * @note The manual says that the reserved bits in the MACAxHR reg should
 * be kept at the same value as they are on reset, but we're not doing that
 * right now... this could be breaking things later.
 */
void EthMAC::setMACAddr(int slot, uint8_t *address, bool enable) {
	uint32_t macRegHigh = 0, macRegLow = 0;

	// fill the low register
	macRegLow |= address[5];
	macRegLow |= (address[4] << 8);
	macRegLow |= (address[3] << 16);
	macRegLow |= (address[2] << 24);

	// fill the high register
	macRegHigh |= address[1];
	macRegHigh |= (address[0] << 8);

	// set the enable bit as needed
	if(enable) {
		macRegHigh |= ETH_MACA1HR_AE;
	}

	// write to the correct register
	if(slot == 0) {
		uint32_t high = ETH->MACA0HR;
		high &= 0x7FFF0000; // keep the reserved bits [30..16]
		high |= 0x80000000; // always set the MO bit

		high |= (macRegHigh & 0x0000FFFF);

		ETH->MACA0LR = macRegLow;
		ETH->MACA0HR = high;
	} else if(slot == 1) {
		uint32_t high = ETH->MACA1HR;
		high &= 0x00FF0000; // keep the reserved bits [23..16]

		high |= (macRegHigh & 0xFF00FFFF);

		ETH->MACA1LR = macRegLow;
		ETH->MACA1HR = high;
	} else if(slot == 2) {
		uint32_t high = ETH->MACA2HR;
		high &= 0x00FF0000; // keep the reserved bits [23..16]

		high |= (macRegHigh & 0xFF00FFFF);

		ETH->MACA2LR = macRegLow;
		ETH->MACA2HR = high;
	} else if(slot == 3) {
		uint32_t high = ETH->MACA3HR;
		high &= 0x00FF0000; // keep the reserved bits [23..16]

		high |= (macRegHigh & 0xFF00FFFF);

		ETH->MACA3LR = macRegLow;
		ETH->MACA3HR = high;
	}
}

/**
 * Changes the state of promiscuous mode. When enabled, all frames are passed
 * to the application, whether they pass the MAC filter or not.
 *
 * When enabled, the RA bit in MACFFR is set.
 */
void EthMAC::setPromiscuousMode(bool enable) {
	uint32_t frameFilter = ETH->MACFFR;

	if(enable) {
		frameFilter |= (ETH_MACFFR_RA | ETH_MACFFR_PM);
	} else {
		frameFilter &= ~(ETH_MACFFR_RA | ETH_MACFFR_PM);
	}

	ETH->MACFFR = frameFilter;
}

} /* namespace net */
