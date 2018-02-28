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

// assert PE0 when writing a packet
#define ASSERT_PE0_FOR_TX					1

namespace net {

#ifdef DEBUG
void EthMACDebugTimerCallback(TimerHandle_t timer) {
	void *ctx = pvTimerGetTimerID(timer);
	EthMAC *mac = static_cast<EthMAC *>(ctx);

	mac->dbgCheckDMAStatus();
	ETH->DMARPDR = ETH_DMARPDR_RPD;
}
#endif

/**
 * Initializes the MAC.
 */
EthMAC::EthMAC(Network *_net, bool useRMII) : net(_net), rmii(useRMII) {
	// set up the RMII state
	/*if(useRMII) {
		LOG(S_INFO, "Setting RMII bit in AFIO mapper");

		RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);

		AFIO->MAPR |= AFIO_MAPR_MII_RMII_SEL;

		RCC_AHBPeriphResetCmd(RCC_AHBPeriph_ETH_MAC, DISABLE);
	}*/

	// enable clocks for the MAC and its DMA engine
	this->setUpClocks();

	// allocate locks
	this->rxDescriptorLock = xSemaphoreCreateBinary();
	xSemaphoreGive(this->rxDescriptorLock);

	this->txDescriptorLock = xSemaphoreCreateBinary();
	xSemaphoreGive(this->txDescriptorLock);

	// clear buffers
	memset((void *) &this->dmaReceivedFramesReady, (int) false,
			EthMAC::dmaReceivedFramesReadySz);

	// set up the task
	this->setUpTransmitTask();

	// reset the MAC
	this->reset();

	// set up registers
	this->setUpMACRegisters();
//	this->setUpMMCRegisters();
	this->setUpDMARegisters();

	// enable interrupts
	this->enableEthernetIRQ();

	// enable the PE0 GPIO if needed
#if ASSERT_PE0_FOR_TX
	GPIO_InitTypeDef gpio;

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOE, &gpio);
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);
#endif

	// XXX: Debugging
/*	TimerHandle_t t = xTimerCreate("MACDbg", pdMS_TO_TICKS(5000), pdTRUE, this, EthMACDebugTimerCallback);
	xTimerStart(t, portMAX_DELAY);
*/
}

/**
 * Configures the MAC for the specified duplex and speed settings.
 */
void EthMAC::linkStateChanged(bool linkUp, bool duplex, net_link_speed_t speed) {
	uint32_t cfg = ETH->MACCR;

	// if the link went down, disable reception and transmission
	if(!linkUp) {
		cfg &= ~(ETH_MACCR_TE); // enable transmitter
		cfg &= ~(ETH_MACCR_RE); // enable receiver

		ETH->MACCR = cfg;
		return;
	}

	// set duplex state
	if(duplex) {
		cfg |= ETH_MACCR_DM; // enable full duplex
	} else {
		cfg &= ~(ETH_MACCR_DM); // disable full duplex
	}

	// set speed
	if(speed == 100) {
		cfg |= ETH_MACCR_FES; // enable fast ethernet
	} else if(speed == 10) {
		cfg &= ~(ETH_MACCR_FES); // disable fast ethernet
	} else {
		LOG(S_ERROR, "Unsupported speed: %u", speed);
	}

	// enable reception and transmission
	if(linkUp) {
		cfg |= ETH_MACCR_TE; // enable transmitter
		cfg |= ETH_MACCR_RE; // enable receiver

		ETH->MACCR = cfg;

		// enable RX and TX DMA
		ETH->DMAOMR |= ETH_DMAOMR_SR;
		ETH->DMAOMR |= ETH_DMAOMR_ST;
	}

	// write status back
	ETH->MACCR = cfg;
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
//	cfg |= ETH_MACCR_APCS; // strip the padding and CRC when receiving frames

//	cfg |= ETH_MACCR_TE; // enable transmitter
//	cfg |= ETH_MACCR_RE; // enable receiver

	ETH->MACCR = cfg;

	// initialize the frame filter
	uint32_t frameFilter = ETH->MACFFR;
	frameFilter &= 0x7FFFF800; // keep reserved bits

//	frameFilter |= ETH_MACFFR_RA; // Pass all (XXX: for debugging)

	frameFilter |= ETH_MACFFR_HPF; // enable hash/perfect filtering
	frameFilter |= ETH_MACFFR_PAM; // pass all multicast packets

	ETH->MACFFR = frameFilter;

	// clear the MAC hash table
	ETH->MACHTHR = 0;
	ETH->MACHTLR = 0;

	// configure flow control
	uint32_t flow = ETH->MACFCR;
	flow &= 0x0000FF00; // keep reserved bits

	flow |= (0x7FFF << 16); // set pause time (1 = 512 bit times)
	flow |= ETH_MACFCR_PLT_Minus28; // re-transmit pause frame 144 slots later

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

	wakeup |= ETH_MACPMTCSR_WFFRPR;
//	wakeup |= ETH_MACPMTCSR_GU;

	ETH->MACPMTCSR = wakeup;

	// mask interrupts from the time stamp trigger
	uint16_t irqMask = (uint16_t) ETH->MACIMR;
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
	// enable clocks for the Ethernet peripheral and its DMA engines
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Tx, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	// enable clock to the CRC engine
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
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

	// delete task and queue
	if(this->transmitTask) {
		vTaskDelete(this->transmitTask);
	}
	if(this->transmitQueue) {
		vQueueDelete(this->transmitQueue);
	}
	if(this->txCompleteSignal) {
		vSemaphoreDelete(this->txCompleteSignal);
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
	macRegLow |= address[0];
	macRegLow |= (address[1] << 8);
	macRegLow |= (address[2] << 16);
	macRegLow |= (address[3] << 24);

	// fill the high register
	macRegHigh |= address[4];
	macRegHigh |= (address[5] << 8);

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

	dmabmr |= ETH_DMABMR_USP; // TX and RX have separate beat settings
	dmabmr |= ETH_DMABMR_RTPR_2_1; // RX DMA gets 2x more bus time than TX DMA
	dmabmr |= ETH_DMABMR_RDP_32Beat; // 32 beats per DMA for RX
	dmabmr |= ETH_DMABMR_PBL_32Beat; // 32 beats per DMA for TX
//	dmabmr |= 0; // descriptor skip length is zero

	ETH->DMABMR = dmabmr;

/*	// flush the TX FIFO
	ETH->DMAOMR |= ETH_DMAOMR_FTF;

	volatile int timeout = 20000;

	do {
		if((ETH->DMAOMR & ETH_DMAOMR_FTF) == 0) {
			break;
		}
	} while(timeout--);

	if(timeout <= 0) {
		LOG(S_ERROR, "Timeout waiting for DMAOMR_FTF to clear");
	}*/


	// configure interrupts
	uint32_t dmaier = ETH->DMAIER;
	dmaier &= 0xFFFE1800; // keep reserved bits

	dmaier |= ETH_DMAIER_NISE; // enable normal interrupts
	dmaier |= ETH_DMAIER_RIE; // receive interrupt enabled
	dmaier |= ETH_DMAIER_TIE; // transmit interrupt enabled
	dmaier |= ETH_DMAIER_ERIE; // early receive interrupt

	dmaier |= ETH_DMAIER_AISE; // enable error interrupts
	dmaier |= ETH_DMAIER_FBEIE; // fatal bus error enabled
	dmaier |= ETH_DMAIER_RPSIE; // receive process stopped
	dmaier |= ETH_DMAIER_TPSIE; // transmit process stopped
	dmaier |= ETH_DMAIER_RBUIE; // receive buffer unavailable
	dmaier |= ETH_DMAIER_TBUIE; // tx buffers unavailable

	ETH->DMAIER = dmaier;


	// configure operation mode (this must be last)
	uint32_t dmaomr = ETH->DMAOMR;
	dmaomr &= 0xF8CD1F21; // keep reserved bits

	dmaomr |= ETH_DMAOMR_DTCEFD; // don't drop frames with payload checksum errors
	dmaomr |= ETH_DMAOMR_RSF; // use receive store-and-forward mode
	dmaomr |= ETH_DMAOMR_TSF; // use transmit store-and-forward mode
	dmaomr |= ETH_DMAOMR_TTC_32Bytes; // tx after 32 bytes in FIFO
	dmaomr |= ETH_DMAOMR_RTC_32Bytes; // rx after 32 bytes in FIFO

//	dmaomr |= ETH_DMAOMR_OSF; // operate on second frame

	ETH->DMAOMR = dmaomr;


	// set up the transmit tx descriptors
	memset((void *) &this->txDescriptor, 0, sizeof(mac_tx_dma_descriptor_t));
	memset((void *) &this->txDescriptor2, 0, sizeof(mac_tx_dma_descriptor_t));

	this->txDescriptor.status |= TX_STATUS_DMA_IRQ_ON_COMPLETE; // IRQ on complete
	this->txDescriptor.status |= TX_STATUS_DMA_NEXT_CHAINED; // next descriptor is chained
	this->txDescriptor.status |= TX_STATUS_DMA_END_OF_LIST; // end of DMA descriptor list

	this->txDescriptor.buf2Address = (uint32_t) &this->txDescriptor;


	this->txDescriptor2.status |= TX_STATUS_DMA_NEXT_CHAINED; // next descriptor is chained
	this->txDescriptor2.status |= TX_STATUS_DMA_IRQ_ON_COMPLETE; // IRQ on complete
	this->txDescriptor2.status |= TX_STATUS_DMA_END_OF_LIST; // end of DMA descriptor list

	this->txDescriptor2.buf2Address = (uint32_t) &this->txDescriptor;


	ETH->DMATDLAR = (uint32_t) &(this->txDescriptor);
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

	// disable transmit DMA
	ETH->DMAOMR &= ~(ETH_DMAOMR_ST);
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
		LOG(S_DEBUG, "Aligned RX descriptors to 0x%08x", address);
	}

	this->rxDescriptors = (mac_rx_dma_descriptor_t *) address;

	// populate each receive descriptor
	for(unsigned int i = 0; i < numBufs; i++) {
		volatile mac_rx_dma_descriptor_t *current = &(this->rxDescriptors[i]);
		volatile mac_rx_dma_descriptor_t *next = &(this->rxDescriptors[(i + 1)]);

		// mark this buffer as ready
		this->dmaReceivedFramesReady[i] = true;

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

	// disable receive DMA
//	ETH->DMAOMR &= ~(ETH_DMAOMR_SR);

	// set the address of the receive descriptors
	this->rxLastReceived = this->rxDescriptors;
	ETH->DMARDLAR = (uint32_t) this->rxDescriptors;

	// re-enable receive DMA and poll for buffers
	ETH->DMAOMR |= ETH_DMAOMR_SR;
	ETH->DMARPDR = ETH_DMARPDR_RPD;

//done: ;
	// release the lock
	xSemaphoreGive(this->rxDescriptorLock);

//	LOG(S_DEBUG, "DMAOMR: 0x%08x", ETH->DMAOMR);
}

/**
 * Releases a received packet (and its buffer) back to the DMA engine so that
 * it may use it again for new received packets.
 */
void EthMAC::releaseRxBuffer(int index) {
	// take the RX lock
	if(xSemaphoreTake(this->rxDescriptorLock, portMAX_DELAY) != pdTRUE) {
		LOG(S_ERROR, "Couldn't take rxDescriptorLock");
		return;
	}

	// re-link the descriptors
	this->dmaReceivedFramesReady[index] = true;
	this->relinkRxDescriptors();

	// release the lock again
	xSemaphoreGive(this->rxDescriptorLock);

//	LOG(S_DEBUG, "Released RX descriptor %u", index);
}

/**
 * Re-creates the linkeages between the descriptors.
 */
void EthMAC::relinkRxDescriptors(void) {
	uint32_t start = 0;
	int freeDescriptors = 0;

	for(size_t i = 0; i < this->numRxDescriptors; i++) {
		volatile mac_rx_dma_descriptor_t *desc = &(this->rxDescriptors[i]);

		// is this descriptor available?
		if(this->dmaReceivedFramesReady[i]) {
			start = (uint32_t) desc;

			// increment the counter
			freeDescriptors++;

			// set the OWN bit so the DMA engine owns the buffer
			desc->status = 0;
			desc->status |= RX_STATUS_DMA_OWNS_BUFFER;

			// is this the last buffer?
			if(i == (this->numRxDescriptors - 1)) {
				// set end-of-list bit
				desc->bufSz |= RX_BUFSZ_END_OF_LIST;
			}
			// if not, chain it to the address of the next buffer
			else {
				uint32_t next = 0;

				// find the next free buffer
				for(size_t j = (i + 1); j < this->numRxDescriptors; j++) {
					// is this one free?
					if(this->dmaReceivedFramesReady[j]) {
						// if so, set its address
						next = (uint32_t) &(this->rxDescriptors[j]);
						break;
					}
				}

				// did we find a free next buffer?
				if(next) {
					// chain the buffer
					desc->bufSz |= RX_BUFSZ_NEXT_CHAINED;
					desc->buf2Address = (uint32_t) next;
				}
				// we couldn't find any buffers that were free
				else {
					desc->bufSz |= RX_BUFSZ_END_OF_LIST;
				}
			}
		}
	}

	// if we have less than 4 descriptors available, send a pause frame
	if(freeDescriptors < 4) {
		ETH->MACFCR |= ETH_MACFCR_FCBBPA;
	}

	// if no descriptors were available, send PAUSE frame and disable reception
	if(start == 0) {
		// send pause frame
		ETH->MACFCR |= ETH_MACFCR_FCBBPA;

		// stop reception
		ETH->DMAOMR &= ~(ETH_DMAOMR_SR);
	} else {
		this->rxLastReceived = (volatile mac_rx_dma_descriptor_t *) start;

		// disable reception to write to the DMA descriptor address
		// TODO: mask IRQs here for the "DMA receive stopped"
//		ETH->DMAOMR &= ~(ETH_DMAOMR_SR);

		// set the address, start reception and re-read descriptors
		ETH->DMARDLAR = start;
		ETH->DMAOMR |= ETH_DMAOMR_SR;

		ETH->DMARPDR = ETH_DMARPDR_RPD;
	}
}

/**
 * Counts the number of free RX descriptors, i.e. those that are owned by the
 * DMA engine.
 */
int EthMAC::availableRxDescriptors(void) {
	int free = 0;

	for(size_t i = 0; i < this->numRxDescriptors; i++) {
		if(this->dmaReceivedFramesReady[i]) {
			free++;
		}
	}

	return free;
}

/**
 * Resets all receive buffers to be available again. This would usually be
 * called by a higher level interface if all receive buffers have been
 * exhausted.
 */
void EthMAC::resetReceiveDescriptors(void) {
	// mark all buffers as available
	for(size_t i = 0; i < this->numRxDescriptors; i++) {
		// count the frame as discarded
		if(this->dmaReceivedFramesReady[i] == false) {
			this->dmaReceivedFramesDiscarded++;
		}

		this->dmaReceivedFramesReady[i] = true;
	}

	// re-link descriptors
	this->relinkRxDescriptors();
}

/**
 * Prints the DMA status.
 */
void EthMAC::dbgCheckDMAStatus(void) {
	LOG(S_DEBUG, "Descriptors: TX = 0x%08x, RX = 0x%08x", ETH->DMATDLAR, ETH->DMARDLAR);
	LOG(S_DEBUG, "Available descriptors: RX = %d", this->availableRxDescriptors());

	LOG(S_DEBUG, "Current descriptors: TX = 0x%08x, RX = 0x%08x", ETH->DMACHTDR, ETH->DMACHRDR);
//	LOG(S_DEBUG, "Current TX buf: 0x%08x, current RX buf: 0x%08x", ETH->DMACHTBAR, ETH->DMACHRBAR);

	LOG(S_DEBUG, "Overflow counter: 0x%08x", ETH->DMAMFBOCR);

	LOG(S_DEBUG, "Status: 0x%08x; Bus Mode: 0x%08x; Op Mode: 0x%08x", ETH->DMASR, ETH->DMABMR, ETH->DMAOMR);
	LOG(S_DEBUG, "Interrupts: 0x%08x", ETH->DMAIER);

	// read management counters
	LOG(S_DEBUG, "Received frames: %d, discarded %d", this->dmaReceivedFrames, this->dmaReceivedFramesDiscarded);
	LOG(S_DEBUG, "Transmitted frames: %d", this->dmaTransmittedFrames);
}



/**
 * Transmits a packet consisting of the specified buffer and length.
 */
void EthMAC::transmitPacket(void *buffer, size_t length, uint32_t userData) {
	BaseType_t ok;

	// check parameters
	if(buffer == nullptr || length > EthMAC::MTU) {
		LOG(S_ERROR, "Invalid parameters: buffer 0x%08x, length %u", buffer, length);
		return;
	}

//	LOG(S_DEBUG, "Write request: buffer 0x%x, length %u, userData %u", buffer, length, userData);

	// build the write request
	mac_write_request_t req;

	req.data = buffer;
	req.length = length;

	req.userData = userData;

	// queue it
	ok = xQueueSendToBack(this->transmitQueue, &req, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Write request couldn't be pushed on queue");
	}
}

/**
 * C trampoline to go into the FreeRTOS task.
 */
void _MACTXTaskTrampoline(void *ctx) {
	(static_cast<EthMAC *>(ctx))->transmitTaskEntry();
}

/**
 * Sets up the transmit task.
 */
void EthMAC::setUpTransmitTask(void) {
	BaseType_t ok;

	// allocate the semaphore used to signal the end of a transmission
	this->txCompleteSignal = xSemaphoreCreateMutex();
	xSemaphoreGive(this->txCompleteSignal);

	// create the queue
	this->transmitQueue = xQueueCreate(EthMAC::TransmitQueueDepth, sizeof(mac_write_request_t));

	if(this->transmitQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// now, create the task
	ok = xTaskCreate(_MACTXTaskTrampoline, "MACTX", EthMAC::TransmitTaskStackSize,
					 this, EthMAC::TransmitTaskPriority, &this->transmitTask);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}
}

/**
 * Entry point for the transmit task.
 */
void EthMAC::transmitTaskEntry(void) {
	BaseType_t ok;

	mac_write_request_t req;

	uint32_t userDataLastTx = 0;
	void *bufferLastTx = nullptr;

	while(1) {
		// block on the write IRQ
		ok = xQueueSemaphoreTake(this->txCompleteSignal, portMAX_DELAY);

		if(ok != pdPASS) {
			LOG(S_ERROR, "Couldn't take transmit complete semaphore!");
			continue;
		}


		// notify the stack that the previous frame sent
		if(bufferLastTx != nullptr) {
//			LOG(S_DEBUG, "Finished transmitting buffer with userdata 0x%08x, status 0x%08x", userDataLastTx, this->txDescriptor.status);

//			this->dbgCheckDMAStatus();

			// prepare a message
			network_message_t msg;
			memset(&msg, 0, sizeof(network_message_t));

			msg.type = kNetworkMessageTransmittedFrame;
			msg.data = (uint8_t *) bufferLastTx;
			msg.userData = userDataLastTx;

			// send message
			ok = xQueueSendToBack(this->net->messageQueue, &msg, portMAX_DELAY);

			if(ok != pdPASS) {
				LOG_ISR(S_ERROR, "Couldn't write network task message: queue full");
			}
		}


		// attempt to get a write request
		ok = xQueueReceive(this->transmitQueue, &req, portMAX_DELAY);

		if(ok != pdPASS) {
			LOG(S_ERROR, "Couldn't dequeue from transmit queue");
			continue;
		}

		// store the information on this write so we can notify write completion
		bufferLastTx = req.data;
		userDataLastTx = req.userData;


		// get the lock on the tx descriptor
		xSemaphoreTake(this->txDescriptorLock, portMAX_DELAY);

		// stop transmit state machine (and mask tx stopped irq)
		uint32_t dmaier = ETH->DMAIER;
		ETH->DMAIER &= ~(ETH_DMAIER_TPSIE);

//		ETH->DMAOMR &= ~(ETH_DMAOMR_ST);

		// set the buffer and length we wish to send
		this->txDescriptor.bufSz = (uint32_t) (req.length & 0x1FFF);
		this->txDescriptor.buf1Address = (uint32_t) req.data;

		// set up the descriptor status field
		this->txDescriptor.status |= TX_STATUS_DMA_IRQ_ON_COMPLETE; // IRQ on complete
		this->txDescriptor.status |= TX_STATUS_DMA_LAST_SEGMENT; // last segment
		this->txDescriptor.status |= TX_STATUS_DMA_FIRST_SEGMENT; // first segment
		this->txDescriptor.status |= TX_STATUS_DMA_END_OF_LIST; // end of list
		this->txDescriptor.status |= TX_STATUS_DMA_CIC_ALL; // insert all checksums
		this->txDescriptor.status |= TX_STATUS_DMA_OWNS_BUFFER; // DMA owns buffer

		// force sync
		__DSB();

		// is the TX buffer busy flag set?
		if(ETH->DMASR & ETH_DMASR_TBUS) {
		    ETH->DMASR = ETH_DMASR_TBUS;
		}

		// we're done writing to the descriptor, so release the lock
		xSemaphoreGive(this->txDescriptorLock);

//		LOG(S_DEBUG, "Set up TX descriptor and started TX for userdata %u", userDataLastTx);

		// acknowledge the IRQ if not already done
		ETH->DMASR = ETH_DMASR_TPSS | ETH_DMASR_ETS;

		// re-enable transmitter, if it was disabled
//		ETH->MACCR |= ETH_MACCR_TE;

		// force the DMA engine to poll the buffers again
//		ETH->DMATDLAR = (uint32_t) &(this->txDescriptor);
//		ETH->DMAOMR |= ETH_DMAOMR_ST;
		ETH->DMATPDR = ETH_DMATPDR_TPD;

#if ASSERT_PE0_FOR_TX
		GPIO_SetBits(GPIOE, GPIO_Pin_0);
#endif

		this->dmaTransmitStopped = false;

		// unmask the TX state machine stopped IRQ
		ETH->DMAIER |= (dmaier & ETH_DMAIER_TPSIE);
	}
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
	// was the interrupt caused due to an MMC interrupt?
	if(ETH->DMASR & ETH_DMASR_MMCS) {
		this->handleMMCInterrupt();
	}

	// was it generated due to a power management event?
	 if(ETH->DMASR & ETH_DMASR_PMTS) {
		// TODO: handle power events
		(void) ETH->MACPMTCSR;

		ETH->DMASR |= ETH_DMASR_PMTS;
	}

	// check if the DMA interrupt was due to normal operation
	if(ETH->DMASR & ETH_DMASR_NIS) {
		this->handleDMAInterrupt(ETH->DMASR);
	}

	// check if the DMA interrupt was caused due to an error
	if(ETH->DMASR & ETH_DMASR_AIS) {
		this->handleDMAErrorInterrupt(ETH->DMASR);
	}
}

/**
 * Handles an MMC interrupt.
 */
void EthMAC::handleMMCInterrupt(void) {
	// reading the MMC counters clears their interrupts
	this->readMMCCounters();

	// acknowledge the interrupt here
	ETH->DMASR = ETH_DMASR_MMCS;
}

/**
 * Handles an interrupt generated by the DMA engine, calling into the network
 * stack as needed.
 */
void EthMAC::handleDMAInterrupt(uint32_t dmasr) {
	BaseType_t ok = pdFALSE, woke = pdFALSE;

	// set up a message, in case we need to pass it to the network stack
	network_message_t msg;

	// acknowledge all interrupts
//	ETH->DMASR |= 0xFFFFFFFF;

	// was this an early receive interrupt?
	if(dmasr & ETH_DMASR_ERS) {
		// copy address of descriptor
		volatile uint32_t addr = ETH->DMACHRDR;
		this->rxLastReceived = (volatile mac_rx_dma_descriptor_t *) addr;

		// acknowledge interrupt
		ETH->DMASR = ETH_DMASR_ERS;
	}
	// was this a receive interrupt?
	else if(dmasr & ETH_DMASR_RS) {
		uint32_t index = this->indexOfLastReceivedISR();
		uint32_t status = this->rxDescriptors[index].status;

//		(volatile void) index;

		// fill in the message and send it
		msg.type = kNetworkMessageReceivedFrame;
		msg.index = index;
		msg.data = (uint8_t *) this->rxDescriptors[index].buf1Address;
		msg.packetLength = (status & RX_STATUS_DMA_FRAME_LENGTH_MASK) >> RX_STATUS_DMA_FRAME_LENGTH_SHIFT;

		// send the message
		ok = xQueueSendToBackFromISR(this->net->messageQueue, &msg, &woke);

		if(ok != pdPASS) {
			this->discardLastRxPacket();
//			LOG_ISR(S_ERROR, "Couldn't write network task message: queue full");
		} else {
			this->dmaReceivedFrames++;

			// copy the address of the next descriptor for later
			volatile uint32_t addr = ETH->DMACHRDR;
			this->rxLastReceived = (volatile mac_rx_dma_descriptor_t *) addr;

			// mark the frame as used
			this->dmaReceivedFramesReady[index] = false;
			this->relinkRxDescriptors();
		}

		// acknowledge interrupt
		ETH->DMASR = ETH_DMASR_RS;
	}

	// was this a transmit interrupt?
	else if(dmasr & ETH_DMASR_TS) {
		// signal the end of a transmission
		xSemaphoreGiveFromISR(this->txCompleteSignal, &woke);
		ok = pdPASS;

#if ASSERT_PE0_FOR_TX
		GPIO_ResetBits(GPIOE, GPIO_Pin_0);
#endif

		// increment counter
		this->dmaTransmittedFrames++;

//		LOG_ISR(S_DEBUG, "TX descriptor status: 0x%08x", this->txDescriptor.status);

		// acknowledge interrupt
		ETH->DMASR = ETH_DMASR_TS;
	}
	// are all transmit buffers unavailable?
	else if(dmasr & ETH_DMASR_TBUS) {
		this->dmaTransmitStopped = true;

		// acknowledge interrupt
		ETH->DMASR = ETH_DMASR_TBUS;
	}
	// early transmit?
	else if(dmasr & ETH_DMASR_ETS) {
		ETH->DMASR = ETH_DMASR_ETS;
	}

	// acknowledge the normal irqs
	ETH->DMASR = ETH_DMASR_NIS;


	// if a message was sent, perform a context switch if applicable
	if(ok == pdPASS) {
		portYIELD_FROM_ISR(woke);
	}
}

/**
 * If the interrupt was generated in response to a DMA error, handle it here.
 */
void EthMAC::handleDMAErrorInterrupt(uint32_t dmasr) {
	BaseType_t ok = pdFALSE, woke = pdFALSE;

	// mask error value
	uint32_t err = (dmasr & 0x0000BFBE);

	// set up a message, in case we need to pass it to the network stack
	network_message_t msg;

	// are the receive buffers unavailable?
	if(err & ETH_DMASR_RBUS) {
		this->dmaReceiveStopped = true;

		// send message
		msg.type = kNetworkMessageRxPacketLost;

		msg.index = 0;
		ok = xQueueSendToBackFromISR(this->net->messageQueue, &msg, &woke);

		if(ok != pdPASS) {
			this->discardLastRxPacket();
//			LOG_ISR(S_ERROR, "Couldn't write network task message: queue full");
		} else {
			// we should have frames ready to receive so force DMA to poll
			ETH->DMARPDR = ETH_DMARPDR_RPD;
		}

		// clear interrupt
		ETH->DMASR = ETH_DMASR_RBUS;
	}
	// did the receive process enter the stopped state?
	else if(err & ETH_DMASR_RPSS) {
		this->dmaReceiveStopped = true;

		// send message
		msg.type = kNetworkMessageRxPacketLost;

		msg.index = 1;
		ok = xQueueSendToBackFromISR(this->net->messageQueue, &msg, &woke);

		if(ok != pdPASS) {
			this->discardLastRxPacket();
//			LOG_ISR(S_ERROR, "Couldn't write network task message: queue full");
		}

		// clear interrupt
		ETH->DMASR = ETH_DMASR_RPSS;
	}
	// are the transmit buffers unavailable?
	else if(err & ETH_DMASR_TBUS) {
		this->dmaTransmitStopped = true;

		LOG_ISR(S_ERROR, "All tx buffers unavailable");

		// clear interrupt
		ETH->DMASR = ETH_DMASR_TBUS;
	}
	// was the TX process stopped? (we get this from clearing ST)
	else if(err & ETH_DMASR_TPSS) {
		this->dmaTransmitStopped = true;

//		LOG_ISR(S_ERROR, "TX process stopped: 0x%08x", ETH->DMASR);

		// clear interrupt
		ETH->DMASR = ETH_DMASR_TPSS;
	}
	// handle unknown error
	else {
		// clear all interrupts
		ETH->DMASR = 0xFFFFFFFF;

		LOG_ISR(S_ERROR, "DMA error: 0x%04x", err);
	}

	// send message (DEBUG)
	if(ok != pdPASS) {
		msg.type = kNetworkMessageDebugUnknownIRQ;
		msg.index = err;
		ok = xQueueSendToFrontFromISR(this->net->messageQueue, &msg, &woke);
	}


	// if a message was sent, perform a context switch if applicable
	if(ok == pdPASS) {
		portYIELD_FROM_ISR(woke);
	}
}

/**
 * Returns the index of the descriptor of the last received packet. This does
 * some simple pointer arithmetic on the pointer written during the "early
 * receive" interrupt.
 */
uint32_t EthMAC::indexOfLastReceivedISR(void) {
	// figure out the index in the buffer
	uint32_t addr = (uint32_t) this->rxLastReceived;
	uint32_t start = (uint32_t) this->rxDescriptors;

	uint32_t difference = (addr - start);

	return (difference / sizeof(mac_rx_dma_descriptor_t));
}

/**
 * Discards the last received packet.
 */
void EthMAC::discardLastRxPacket(void) {
	// mark the last received frame as already handled
	uint32_t index = this->indexOfLastReceivedISR();
	this->dmaReceivedFramesReady[index] = true;

	// re-create descriptor linkage
	this->relinkRxDescriptors();

	// increment dropped frames counter
	this->dmaReceivedFramesDiscarded++;
}

} /* namespace net */
