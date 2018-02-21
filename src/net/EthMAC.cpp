/*
 * EthMAC.cpp
 *
 *  Created on: Feb 19, 2018
 *      Author: tristan
 */
#define LOG_MODULE "MAC"

#include "EthMAC.h"
#include "EthMACPrivate.h"

#include "Network.h"

#include <LichtensteinApp.h>

#include <cstring>

namespace net {

/**
 * Initializes the MAC.
 */
EthMAC::EthMAC(Network *_net, bool useRMII) : net(_net), rmii(useRMII) {
	// set up the RMII state
	if(useRMII) {
		LOG(S_INFO, "Setting RMII bit in AFIO mapper");

		RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);

		AFIO->MAPR |= AFIO_MAPR_MII_RMII_SEL;

		RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
	}

	// enable clocks for the MAC and its DMA engine
	this->setUpClocks();

	// allocate locks
	this->rxDescriptorLock = xSemaphoreCreateBinary();
	xSemaphoreGive(this->rxDescriptorLock);

	this->txDescriptorLock = xSemaphoreCreateBinary();
	xSemaphoreGive(this->txDescriptorLock);

	// reset the MAC
//	this->reset();

	// set up registers
	this->setUpMACRegisters();
	this->setUpMMCRegisters();
	this->setUpDMARegisters();

	// enable interrupts
	this->enableEthernetIRQ();
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

	ETH->MACCR = cfg;

	// initialize the frame filter
	uint32_t frameFilter = ETH->MACFFR;
	frameFilter &= 0x7FFFF800; // keep reserved bits

	frameFilter |= ETH_MACFFR_RA; // Pass all (XXX: for debugging)

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
 * Resets the Ethernet controller.
 */
void EthMAC::reset(void) {
	// set the software reset bit
	ETH->DMABMR = ETH_DMABMR_SR;

	// wait for the bit to clear
	volatile int timeout = 1000000;

	do {
		// check that the bit has cleared to zero
		if((ETH->DMABMR & ETH_DMABMR_SR) == 0) {
			return;
		}
	} while(timeout--);

	// we timed out
	LOG(S_ERROR, "MAC reset timeout, DMABMR = 0x%08x", ETH->DMABMR);
}



/**
 * Sets up the clocks for the Ethernet peripheral.
 */
void EthMAC::setUpClocks(void) {
	// enable clocks for the ETH peripherals
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	// de-assert MAC reset
//	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC_Tx, DISABLE);
//	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC_Rx, DISABLE);
//	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);

	// reset the MAC
//	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
//	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
}

/**
 * De-initializes the MAC by turning off its clocks.
 */
EthMAC::~EthMAC() {
	// disable interrupts
	this->disableEthernetIRQ();

	// terminate DMA
	this->shutDownDMA();

	// reset the MAC registers
	this->reset();

	// turn off clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, DISABLE);

	// reset the MAC
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
	RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);

	// de-allocate locks
	if(this->rxDescriptorLock) {
		vSemaphoreDelete(this->rxDescriptorLock);
	}

	if(this->txDescriptorLock) {
		vSemaphoreDelete(this->txDescriptorLock);
	}
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
 * Reads the 32-bit identifier for a PHY by reading the PHYSID register.
 */
uint32_t EthMAC::readPHYId(uint16_t address) {
	uint32_t reg = 0;
	int temp;

	// read the top half of the register (0x02)
	temp = this->mdioRead(address, 0x02);

	if(temp < 0) {
		LOG(S_ERROR, "Couldn't read PHYSID1 from %d", address);
		return 0xFFFFFFFF;
	}

	reg |= ((temp & 0x0000FFFF) << 16);

	// read the lower half of the register (0x03)
	temp = this->mdioRead(address, 0x03);

	if(temp < 0) {
		LOG(S_ERROR, "Couldn't read PHYSID2 from %d", address);
		return 0xFFFFFFFF;
	}

	reg |= (temp & 0x0000FFFF);

	// we're done, return the register
	return reg;
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



/**
 * Configures the DMA engine's registers.
 */
void EthMAC::setUpDMARegisters(void) {
	// configure DMA bus mode
	uint32_t dmabmr = ETH->DMABMR;
	dmabmr &= 0xFC000080; // keep reserved bits

	dmabmr |= ETH_DMABMR_RTPR_2_1; // RX DMA gets 2x more bus time than TX DMA
	dmabmr |= ETH_DMABMR_RDP_32Beat; // 32 beats per DMA for RX
	dmabmr |= ETH_DMABMR_PBL_32Beat; // 32 beats per DMA for TX
//	dmabmr |= 0; // descriptor skip length is zero

	ETH->DMABMR = dmabmr;

	LOG(S_DEBUG, "DMABMR: 0x%08x", ETH->DMABMR);

	// configure operation mode
	uint32_t dmaomr = ETH->DMAOMR;
	dmaomr &= 0xF8CD1F21; // keep reserved bits

	dmaomr |= ETH_DMAOMR_DTCEFD; // don't drop payload checksum errors
	dmaomr |= ETH_DMAOMR_RSF; // use receive store-and-forward mode
	dmaomr |= ETH_DMAOMR_TSF; // use transmit store-and-forward mode

//	dmaomr |= ETH_DMAOMR_FTF; // flush TX FIFO

	ETH->DMAOMR = dmaomr;


/*	// wait for the flush TX FIFO bit to be cleared
	int timeout = 100000;
	bool timeoutExpired = true;

	do {
		// read register and test the FTF bit
		dmaomr = ETH->DMAOMR;

		if((dmaomr & ETH_DMAOMR_FTF) == 0) {
			// bit is clear so the flush is complete
			timeoutExpired = false;
			break;
		}
	} while(timeout--);

	if(timeoutExpired) {
		LOG(S_ERROR, "Timeout waiting for ETH_DMAOMR_FTF clear");
	}*/


	// configure interrupts
	uint32_t dmaier = ETH->DMAIER;
	dmaier &= 0xFFFE1800; // keep reserved bits

	dmaier |= ETH_DMAIER_NISE; // enable normal interrupts
	dmaier |= ETH_DMAIER_RIE; // receive interrupt enabled
	dmaier |= ETH_DMAIER_TIE; // transmit interrupt enabled

	dmaier |= ETH_DMAIER_AISE; // enable error interrupts
	dmaier |= ETH_DMAIER_FBEIE; // fatal bus error enabled
	dmaier |= ETH_DMAIER_RPSIE; // receive process stopped
	dmaier |= ETH_DMAIER_TPSIE; // transmit process stopped
	dmaier |= ETH_DMAIER_RBUIE; // receive buffer unavailable

	ETH->DMAIER = dmaier;
}

/**
 * Shuts down the DMA engine and de-allocates descriptors.
 */
void EthMAC::shutDownDMA(void) {
	// disable receive DMA and de-allocate buffers
	ETH->DMAOMR &= ~(ETH_DMAOMR_SR);

	if(this->rxDescriptorsMem) {
		vPortFree(this->rxDescriptorsMem);
		this->rxDescriptorsMem = nullptr;
	}

	// disable transmit DMA and de-allocate buffers
	ETH->DMAOMR &= ~(ETH_DMAOMR_ST);

	if(this->txDescriptorsMem) {
		vPortFree(this->txDescriptorsMem);
		this->txDescriptorsMem = nullptr;
	}
}

/**
 * Forces the transmit DMA to resume transmission.
 */
void EthMAC::resumeTxDMA(void) {
	ETH->DMATPDR = ETH_DMATPDR_TPD;
}

/**
 * Forces the receive DMA to resume receiving frames.
 */

void EthMAC::resumeRxDMA(void) {
	ETH->DMARPDR = ETH_DMARPDR_RPD;
}

/**
 * Sets the specified buffers as receive buffers. Packets are written into
 * these buffers as they are received. The application should release each
 * buffer as it finishes processing its contents so the Ethernet peripheral
 * is never starved of buffers.
 */
void EthMAC::setRxBuffers(void *buffers, size_t numBufs) {
	uint32_t *bufferAddresses = static_cast<uint32_t*>(buffers);

	// take the RX lock
	if(xSemaphoreTake(this->rxDescriptorLock, portMAX_DELAY) != pdTRUE) {
		LOG(S_ERROR, "Couldn't take rxDescriptorLock");
	}

	// disable receive DMA
	ETH->DMAOMR &= ~(ETH_DMAOMR_SR);

	// deallocate any old RX descriptors
	if(this->rxDescriptorsMem) {
		vPortFree(this->rxDescriptorsMem);
		this->rxDescriptorsMem = nullptr;
	}

	// allocate buffer for the RX descriptors and align it
	size_t rxDescBufSz = sizeof(mac_rx_dma_descriptor_t) * numBufs;

	this->rxDescriptorsMem = (mac_rx_dma_descriptor_t *) pvPortMalloc(rxDescBufSz + 16);
	LOG(S_DEBUG, "Allocated %d RX descriptors at 0x%x", numBufs, this->rxDescriptorsMem);

	memset(this->rxDescriptorsMem, 0, rxDescBufSz + 16);

	// align the buffer
	uint32_t address = (uint32_t) this->rxDescriptorsMem;
	uint32_t lowNybble = address & 0x0000000F;

	if(lowNybble) {
		address += (0x10 - lowNybble);
		LOG(S_DEBUG, "Aligned RX buffers to 0x%08x", address);
	}

	this->rxDescriptors = (mac_rx_dma_descriptor_t *) address;

	// populate each receive descriptor
	for(unsigned int i = 0; i < numBufs; i++) {
		mac_rx_dma_descriptor_t *current = &(this->rxDescriptors[i]);
		mac_rx_dma_descriptor_t *next = &(this->rxDescriptors[(i + 1)]);

		// configure the size and address
		current->bufSz = ((EthMAC::rxBufSize) & 0x1FFF);
		current->buf1Address = bufferAddresses[i];

		// is this the last buffer?
		if(i == (numBufs - 1)) {
			// if so, set the end-of-list bit
			current->bufSz |= RX_BUFSZ_END_OF_LIST;
		}
		// if not, chain it to the address of the next buffer
		else {
			current->bufSz |= RX_BUFSZ_NEXT_CHAINED;
			current->buf2Address = (uint32_t) next;
		}

		// this buffer belongs to the DMA engine
		current->status |= RX_STATUS_DMA_OWNS_BUFFER;

//		LOG(S_DEBUG, "Set up descriptor %u: address 0x%x (next 0x%x) size 0x%x, status 0x%x", i, current->buf1Address, current->buf2Address, current->bufSz, current->status);
	}

	this->numRxDescriptors = numBufs;


	// set the address of the receive descriptors
	ETH->DMARDLAR = (uint32_t) this->rxDescriptors;

	// re-enable receive DMA and poll for buffers
	ETH->DMAOMR |= ETH_DMAOMR_SR;

	this->resumeRxDMA();

done: ;
	// release the lock
	xSemaphoreGive(this->rxDescriptorLock);

	LOG(S_DEBUG, "DMAOMR: 0x%08x", ETH->DMAOMR);
}

/**
 * Finds a receive buffer that's owned by the CPU in the DMA descriptor list,
 * and returns its information. If no frames have been received, false is
 * returned.
 */
bool EthMAC::getRxPacket(uint8_t **data, size_t *length, unsigned int *bufIndex) {
	bool found = false;
	uint32_t status;

	// take the RX lock
	if(xSemaphoreTake(this->rxDescriptorLock, portMAX_DELAY) != pdTRUE) {
		LOG(S_ERROR, "Couldn't take rxDescriptorLock");
		return false;
	}

	// iterate through the descriptors
	for(size_t i = 0; i < this->numRxDescriptors; i++) {
		status = this->rxDescriptors[i].status;

		// is the DMA owns buffer bit clear?
		if((status & RX_STATUS_DMA_OWNS_BUFFER) == 0) {
			*data = (uint8_t *) this->rxDescriptors[i].buf1Address;
			*length = (status & RX_STATUS_DMA_FRAME_LENGTH_MASK) >> RX_STATUS_DMA_FRAME_LENGTH_SHIFT;
			*bufIndex = i;

			found = true;
			goto done;
		}
	}


done: ;
	// release the lock
	xSemaphoreGive(this->rxDescriptorLock);

	return found;
}

/**
 * Releases a received packet (and its buffer) back to the DMA engine so that
 * it may use it again for new received packets.
 */
void EthMAC::releaseRxPacket(int index) {
	// take the RX lock
	if(xSemaphoreTake(this->rxDescriptorLock, portMAX_DELAY) != pdTRUE) {
		LOG(S_ERROR, "Couldn't take rxDescriptorLock");
		return;
	}

	// set the OWN bit so the DMA engine owns the buffer
	this->rxDescriptors[index].status = 0;
	this->rxDescriptors[index].status |= RX_STATUS_DMA_OWNS_BUFFER;

	// force the DMA engine to poll the descriptors again
	this->resumeRxDMA();

	// release the lock again
	xSemaphoreGive(this->rxDescriptorLock);
}

/**
 * Sets the specified buffers as transmit buffers. Packets to be sent can be
 * written into these buffers, and transmitted over the wire with another
 * call.
 */
void EthMAC::setTxBuffers(void *buffers, size_t numBufs) {
	uint32_t *bufferAddresses = static_cast<uint32_t*>(buffers);

	// take the RX lock
	if(xSemaphoreTake(this->txDescriptorLock, portMAX_DELAY) != pdTRUE) {
		LOG(S_ERROR, "Couldn't take txDescriptorLock");
	}

	// disable receive DMA
	ETH->DMAOMR &= ~(ETH_DMAOMR_ST);

	// deallocate any old TX descriptors
	if(this->txDescriptorsMem) {
		vPortFree(this->txDescriptorsMem);
		this->txDescriptorsMem = nullptr;
	}


	// allocate buffer for the TX descriptors and align it
	size_t txDescBufSz = sizeof(mac_tx_dma_descriptor_t) * numBufs;

	this->txDescriptorsMem = pvPortMalloc(txDescBufSz + 16);
	LOG(S_DEBUG, "Allocated %d TX descriptors at 0x%x", numBufs, this->txDescriptorsMem);

	memset(this->txDescriptorsMem, 0, (txDescBufSz + 16));

	// align the buffer
	uint32_t address = (uint32_t) this->txDescriptorsMem;
	uint32_t lowNybble = address & 0x0000000F;

	if(lowNybble) {
		address += (0x10 - lowNybble);
		LOG(S_DEBUG, "Aligned TX buffers to 0x%08x", address);
	}

	this->txDescriptors = (mac_tx_dma_descriptor_t *) address;


	// populate each receive descriptor
	for(unsigned int i = 0; i < numBufs; i++) {
		mac_tx_dma_descriptor_t *current = &(this->txDescriptors[i]);
		mac_tx_dma_descriptor_t *next = &(this->txDescriptors[(i + 1)]);

		// configure the size and address
		current->bufSz = ((EthMAC::rxBufSize) & 0x1FFF);
		current->buf1Address = bufferAddresses[i];

		// is this the last buffer?
		if(i == (numBufs - 1)) {
			// if so, set the end-of-list bit
			current->status |= TX_STATUS_DMA_END_OF_LIST;
		}
		// if not, chain it to the address of the next buffer
		else {
			current->status |= TX_STATUS_DMA_NEXT_CHAINED;
			current->buf2Address = (uint32_t) next;
		}

		// generate an IRQ when the buffer is transmitted
		current->status |= TX_STATUS_DMA_IRQ_ON_COMPLETE;
		// insert IP header and payload checksums
		current->status |= TX_STATUS_DMA_CIC_ALL;

		// the buffer contains a complete frame
		current->status |= TX_STATUS_DMA_FIRST_SEGMENT;
		current->status |= TX_STATUS_DMA_LAST_SEGMENT;

//		LOG(S_DEBUG, "Set up descriptor %u: address 0x%x (next 0x%x) size 0x%x, status 0x%x", i, current->buf1Address, current->buf2Address, current->bufSz, current->status);
	}

	this->numTxDescriptors = numBufs;


	// set the address of the receive descriptors
	ETH->DMATDLAR = (uint32_t) this->txDescriptors;

	// re-enable transmit DMA and poll for buffers
	ETH->DMAOMR |= ETH_DMAOMR_ST;

	this->resumeTxDMA();

done: ;
	// release the lock
	xSemaphoreGive(this->txDescriptorLock);

	LOG(S_DEBUG, "DMAOMR: 0x%08x", ETH->DMAOMR);
}

/**
 * Prints the DMA status.
 */
void EthMAC::dbgCheckDMAStatus(void) {
	// print the address of the TX and RX descriptors
	LOG(S_DEBUG, "TX descriptors: 0x%08x, RX descriptors: 0x%08x", ETH->DMATDLAR, ETH->DMARDLAR);

	LOG(S_DEBUG, "Current TX descriptor: 0x%08x, current RX descriptors: 0x%08x", ETH->DMACHTDR, ETH->DMACHRDR);
	LOG(S_DEBUG, "Current TX buf: 0x%08x, current RX buf: 0x%08x", ETH->DMACHTBAR, ETH->DMACHRBAR);

	LOG(S_DEBUG, "Overflow counter: 0x%08x", ETH->DMAMFBOCR);

	// print status register
	LOG(S_DEBUG, "Status: 0x%08x", ETH->DMASR);
	LOG(S_DEBUG, "Bus Mode: 0x%08x", ETH->DMABMR);
	LOG(S_DEBUG, "Op Mode: 0x%08x", ETH->DMAOMR);

	LOG(S_DEBUG, "Interrupts: 0x%08x", ETH->DMAIER);
}



/**
 * Enables the Ethernet interrupt in the NVIC.
 */
void EthMAC::enableEthernetIRQ(void) {
	NVIC_InitTypeDef nvic;

	// configure the interrupt
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannel = ETH_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4);

	NVIC_Init(&nvic);
}

/**
 * Disables the Ethernet interrupt in the NVIC.
 */
void EthMAC::disableEthernetIRQ(void) {
	NVIC_InitTypeDef nvic;

	// configure the interrupt
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = DISABLE;
	nvic.NVIC_IRQChannel = ETH_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = (configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4);

	NVIC_Init(&nvic);
}

/**
 * Handles an interrupt from the Ethernet peripheral.
 */
void EthMAC::handleIRQ(void) {
	uint32_t dmasr = ETH->DMASR;

	// was the interrupt caused due to an MMC interrupt?
	if(dmasr & ETH_DMASR_MMCS) {
		this->handleMMCInterrupt();
	}

	// was it generated due to a power management event?
	if(dmasr & ETH_DMASR_PMTS) {
		// TODO: handle power events
	}

	// check if the DMA interrupt was caused due to an error
	if(dmasr & ETH_DMASR_AIS) {
		this->handleDMAErrorInterrupt(dmasr);
	}

	// check if the DMA interrupt was due to normal operation
	if(dmasr & ETH_DMASR_NIS) {
		this->handleDMAInterrupt(dmasr);
	}
}

/**
 * Handles an MMC interrupt.
 */
void EthMAC::handleMMCInterrupt(void) {
	// reading the MMC counters clears their interrupts
	this->readMMCCounters();
}

/**
 * Handles an interrupt generated by the DMA engine, calling into the network
 * stack as needed.
 */
void EthMAC::handleDMAInterrupt(uint32_t dmasr) {
	BaseType_t ok = pdFALSE, woke = pdFALSE;

	// set up a message, in case we need to pass it to the network stack
	network_message_t msg;
//	memset(&msg, 0, sizeof(network_message_t));


	// was this a receive interrupt?
	if(dmasr & ETH_DMASR_RS) {
		// fill in the message and send it
		msg.type = kNetworkMessageReceiveInterrupt;
		ok = xQueueSendToFrontFromISR(this->net->messageQueue, &msg, &woke);
	}
	// was this an early receive interrupt?
	if(dmasr & ETH_DMASR_ERS) {
		// TODO: do something here?
	}

	// was this a transmit interrupt?
	if(dmasr & ETH_DMASR_TS) {
		// fill in the message and send it
		msg.type = kNetworkMessageTransmitInterrupt;
		ok = xQueueSendToFrontFromISR(this->net->messageQueue, &msg, &woke);
	}
	// are all transmit buffers unavailable?
	if(dmasr & ETH_DMASR_TBUS) {
		// TODO: do something here?
	}


	// if a message was sent, perform a context switch if applicable
	if(ok == pdPASS) {
		portYIELD_FROM_ISR(woke);
	}
}

/**
 * If the interrupt was generated in response to a DMA error, handle it here.
 */
void EthMAC::handleDMAErrorInterrupt(uint32_t dmasr) {
	LOG_ISR(S_ERROR, "DMA error: 0x%08x", dmasr);
}

} /* namespace net */
