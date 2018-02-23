/*
 * Network.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "NET"

#include "Network.h"

#include "NetworkPrivate.h"
#include "EthMAC.h"
#include "EthPHY.h"

#include "../board/Board.h"
#include "../clock/Clock.h"

#include <LichtensteinApp.h>

#include <cstring>

// hardware config for EEPROM
#if HW == HW_MUSTARD

// use the RMII interface
#define ETH_RMII							1

#endif

static Network *gNetwork = nullptr;

/**
 * Allocates the shared clock.
 */
void Network::init(void) {
	if(!gNetwork) {
		gNetwork = new Network();
	}
}

/**
 * Returns the shared clock instance.
 */
Network *Network::sharedInstance() noexcept {
	return gNetwork;
}

/**
 * Sets up the Ethernet hardware and the TCP/IP stack.
 */
Network::Network() {
	// read the MAC address
	this->readMACFromEEPROM();

	// create the network task
	this->setUpTask();

	// configure PLL for clock output to PHY and the GPIOs
	this->setUpEthernetGPIOs();
	this->setUpClocks();
}

/**
 * Starts various network services, such as DHCP, the NTP server, and so forth.
 */
void Network::startNetServices(void) {
	// start NTP client so we can sync time
	Clock::sharedInstance()->startNTPClient();
}

/**
 * Closes the TCP/IP stack and turns off the Ethernet link.
 */
Network::~Network() {
	// kill the network task
	vTaskDelete(this->task);
	vQueueDelete(this->messageQueue);
	vSemaphoreDelete(this->txBuffersFreeSemaphore);

	// deallocating the MAC and PHY will reset them
	delete this->mac;
	delete this->phy;

	// release the receive/transmit buffers
	for(size_t i = 0; i < Network::numRxBuffers; i++) {
		vPortFree(this->rxBuffers[i]);
	}

	for(size_t i = 0; i < Network::numTxBuffers; i++) {
		vPortFree(this->txBuffers[i]);
	}
}


/**
 * Reads the MAC address from the EEPROM.
 */
void Network::readMACFromEEPROM(void) {
	Board *b = Board::sharedInstance();
	b->configEEPROMRead(&this->macAddress, Network::ethParamMACOffset, 6);

	// debug
	LOG(S_INFO, "MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
				 this->macAddress[0], this->macAddress[1],
				 this->macAddress[2], this->macAddress[3],
				 this->macAddress[4], this->macAddress[5]);
}



/**
 * Configures the internal PLL to output a 25MHz (MII) or 50MHz (RMII) clock
 * on the MCO pin for the PHY.
 */
void Network::setUpClocks(void) {
	int timeout;

	// get the frequencies of clocks
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	// configure PLL3 for 50MHz
	RCC_PREDIV2Config(RCC_PREDIV2_Div5);
	RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
	RCC_PLL2Config(RCC_PLL2Mul_8);

	RCC_PLL3Config(RCC_PLL3Mul_10);

	// enable PLL2 and wait for lock
	RCC_PLL2Cmd(ENABLE);

	timeout = 20000;

	while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) != SET) {
		if(timeout-- == 0) {
			LOG(S_FATAL, "Couldn't get lock on PLL2");
			break;
		}
	}

	// enable PLL3 wait for lock
	RCC_PLL3Cmd(ENABLE);

	timeout = 20000;

	while(RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) != SET) {
		if(timeout-- == 0) {
			LOG(S_FATAL, "Couldn't get lock on PLL3");
			break;
		}
	}

	// output the clock from the PLL
#if ETH_RMII
	// for RMII, output the PLL3 clock
	RCC_MCOConfig(RCC_MCO_PLL3CLK);
#else
	// for MII, just output the external oscillator clock
	RCC_MCOConfig(RCC_MCO_XT1);
#endif


	RCC_GetClocksFreq(&clocks);
	LOG(S_INFO, "ADCCLK: %d, HCLK: %d, PCLK1: %d, PCLK2: %d, SYSCLK: %d", clocks.ADCCLK_Frequency, clocks.HCLK_Frequency, clocks.PCLK1_Frequency, clocks.PCLK2_Frequency, clocks.SYSCLK_Frequency);
}

/**
 * Sets up the clocks for the Ethernet MAC and the various GPIOs used for the
 * RMII interface.
 */
void Network::setUpMAC(void) {
	// initialize the MAC components
	this->mac = new net::EthMAC(this, true); // true for RMII, false for MII

	// set the unicast MAC address
	this->mac->setMACAddr(this->macAddress);
}

/**
 * Sets up the GPIOs used by the Ethernet peripheral.
 */
void Network::setUpEthernetGPIOs(void) {
	GPIO_InitTypeDef gpio;

	// enable clocks for the GPIOs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
				 RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// all pins below are 50MHz
	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	// RMII_REF_CLK (PA1)
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &gpio);

	// RMII_MDIO (PA2)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &gpio);

	// MCO (PA8)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &gpio);

	// RMII_TX_EN (PB11)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &gpio);

	// RMII_TXD[0,1] (PB12, PB13)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOB, &gpio);

	// RMII_MDC (PC1)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &gpio);


	// RMII_RXD[0,1] (PD9, PD10) remap
	GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

//	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOD, &gpio);

	// RMII_CRS_DV (PD8) remap
	GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

//	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOD, &gpio);
}

/**
 * Scans the MDIO bus for any attached PHYs.
 */
void Network::scanForPHYs(void) {
	uint32_t id;

	// scan all PHYs sequentially
	for(uint16_t phyAddr = 0; phyAddr <= 0x1f; phyAddr++) {
		// read its id
		id = this->mac->readPHYId(phyAddr);

		// if the register is mostly 1's, there's no PHY here
		if((id & 0x0000FFFF) == 0x0000FFFF) {
//			LOG(S_VERBOSE, "no phy at %d", phyAddr);
		}
		// otherwise, we've found a PHY. try to instantiate it
		else {
			LOG(S_INFO, "Found PHY at %d: 0x%08x", phyAddr, id);

			this->phy = net::EthPHY::phyForId(this, id, phyAddr, (ETH_RMII ? true : false));

			if(this->phy == nullptr) {
				LOG(S_FATAL, "Couldn't create PHY with PHYID %08x", id);
			}
		}
	}
}

/**
 * Called by the PHY to indicate a link status change. When we get a link up,
 * attempt to acquire an IP address with DHCP.
 */
void Network::_phyLinkStateChange(bool isLinkUp, bool fromISR) {
//	LOG(S_INFO, "Link status: %d", isLinkUp);

	// prepare a message
	network_message_t msg;
	memset(&msg, 0, sizeof(network_message_t));

	msg.type = kNetworkMessageLinkStateChanged;
	msg.index = (isLinkUp == true) ? 1 : 0;

	// send message
	if(fromISR) {
		// TODO: implement ISR-specific message sending
	} else {
		if(this->postMessage(&msg) == false) {
			LOG(S_ERROR, "Couldn't post link state change message");
		}
	}
}



/**
 * Allocates the receive and transmit buffers.
 */
void Network::allocBuffers(void) {
	// allocate the receive buffers
	for(size_t i = 0; i < Network::numRxBuffers; i++) {
		this->rxBuffers[i] = pvPortMalloc(net::EthMAC::rxBufSize);
		memset(this->rxBuffers[i], 0, net::EthMAC::rxBufSize);

//		LOG(S_VERBOSE, "Allocated rx buffer %u at 0x%x", i, this->rxBuffers[i]);
	}

	// register the receive buffers to the DMA engine
	this->mac->setRxBuffers(this->rxBuffers, Network::numRxBuffers);


	// allocate the transmit buffers
	for(size_t i = 0; i < Network::numTxBuffers; i++) {
		this->txBuffers[i] = pvPortMalloc(net::EthMAC::txBufSize);
//		LOG(S_VERBOSE, "Allocated tx buffer %u at 0x%x", i, this->txBuffers[i]);
	}
}



/**
 * C trampoline to go into the FreeRTOS task.
 */
void _NetTaskTrampoline(void *ctx) {
	(static_cast<Network *>(ctx))->taskEntry();
}

/**
 * Sets up the network message handler task.
 */
void Network::setUpTask(void) {
	BaseType_t ok;

	// create the queue
	this->messageQueue = xQueueCreate(Network::messageQueueSize, sizeof(network_message_t));

	if(this->messageQueue == nullptr) {
		LOG(S_FATAL, "Couldn't create message queue!");
	}

	// create tx buffer semaphore
	this->txBuffersFreeSemaphore = xQueueCreateCountingSemaphore(Network::numRxBuffers, Network::numRxBuffers);

	if(this->txBuffersFreeSemaphore == nullptr) {
		LOG(S_FATAL, "Couldn't create tx buffers free semaphore");
	}

	// now, create the task
	ok = xTaskCreate(_NetTaskTrampoline, "Network", Network::TaskStackSize,
					 this, Network::TaskPriority, &this->task);

	if(ok != pdTRUE) {
		LOG(S_FATAL, "Couldn't create task!");
	}
}

/**
 * Posts a message to the network task.
 *
 * @return true if the message was posted, false otherwise.
 *
 * @note This function is NOT ISR safe!
 */
bool Network::postMessage(network_message_t *msg) {
	BaseType_t ok;

	ok = xQueueSendToBack(this->messageQueue, msg, portMAX_DELAY);

	return (ok == pdTRUE) ? true : false;
}

/**
 * Entry point for the network task. Wait for messages to come in on the que,
 * then act upon them.
 */
void Network::taskEntry(void) {
	BaseType_t ok;
	network_message_t msg;

	// set up the MAC and PHY, respectively
	this->setUpMAC();
	this->scanForPHYs();

	// allocate network buffers
	this->allocBuffers();

	// set up the stack and network servers
	this->setUpStack();
	this->startNetServices();

	int messages = 0;

	// start the message loop
	while(1) {
		/*if((messages++ & 0x1F) == 0) {
			this->mac->dbgCheckDMAStatus();
		}*/

		// attempt to receive a message
		ok = xQueueReceive(this->messageQueue, &msg, portMAX_DELAY);

		// parse message
		if(ok == pdTRUE) {
			switch(msg.type) {
				// link state change notification
				case kNetworkMessageLinkStateChanged: {
					net_link_speed_t speed = this->phy->getSpeed();
					bool duplex = this->phy->isFullDuplex();


					bool linkUp = (msg.index == 1) ? true : false;
					LOG(S_INFO, "Link state: %u", linkUp);
					LOG(S_DEBUG, "Speed: %u, duplex %u", speed, duplex);

					break;
				}

				// if a receive packet is lost, reset the receive process
				case kNetworkMessageRxPacketLost:
					if(msg.index == 1) {
						LOG(S_WARN, "Ethernet reception stopped");
					} else {
						LOG(S_WARN, "Exhausted RX buffers");
					}

					this->mac->dbgCheckDMAStatus();
					break;

				// unknown interrupt
				case kNetworkMessageDebugUnknownIRQ:
					LOG(S_INFO, "Unknown IRQ: 0x%04x", msg.index);
					break;

				// received frame
				case kNetworkMessageReceivedFrame:
					this->handleReceivedFrame(&msg);
					break;
				// frame was transmitted, return the buffer back
				case kNetworkMessageTransmittedFrame:
					this->handleTransmittedFrame(&msg);
					break;

				// unhandled message
				default:
					LOG(S_INFO, "Received unhandled message type %u", msg.type);
					break;
			}
		}
		// handle receive errors
		else {
			LOG(S_ERROR, "Couldn't receive from message queue: %u", ok);
		}
	}
}

/**
 * Handles a received frame by forwarding it to the FreeRTOS TCP/IP stack.
 */
void Network::handleReceivedFrame(network_message_t *msg) {
//	LOG(S_DEBUG, "Received frame of length %u, index %u", msg->packetLength, msg->index);

	// TODO: forward to the IP stack

	// release the packet
	this->mac->releaseRxBuffer(msg->index);
}

/**
 * Handles a transmitted frame by releasing the transmit buffer back to the
 * network stack.
 */
void Network::handleTransmittedFrame(network_message_t *msg) {
	unsigned int index = msg->userData;

	// user data is just the buffer index; mark it as free
	this->txBuffersFree[index] = true;

	// increment semaphore
	xSemaphoreGive(this->txBuffersFreeSemaphore);
}

/**
 * Attempts to acquire a transmit buffer. If no buffers are available, this
 * routine blocks until one can be acquired.
 */
void *Network::getTxBuffer(size_t size) {
	BaseType_t ok;
	unsigned int freeBuffer = 0xFFFFFFFF;

	// make sure the size doesn't exceed the buffer size
	if(size > net::EthMAC::txBufSize) {
		LOG(S_ERROR, "Size too large, %u bytes", size);
		return nullptr;
	}

	// attempt to take the semaphore
	ok = xSemaphoreTake(this->txBuffersFreeSemaphore, portMAX_DELAY);

	if(ok != pdPASS) {
		LOG(S_ERROR, "Couldn't take tx buffers semaphore");
		return nullptr;
	}


	// find a free buffer
	for(unsigned int i = 0; i < Network::numTxBuffers; i++) {
		// is this buffer free?
		if(this->txBuffersFree[i]) {
			freeBuffer = i;
			break;
		}
	}

	// error checking
	if(freeBuffer == 0xFFFFFFFF) {
		LOG(S_ERROR, "Couldn't find free buffer after taking semaphore");
		return nullptr;
	}


	// mark the buffer as used and return it
	this->txBuffersFree[freeBuffer] = true;

	return this->txBuffers[freeBuffer];
}



/**
 * Sets up the TCP/IP stack.
 */
void Network::setUpStack(void) {

}



/**
 * When the Ethernet IRQ is fired, call into the MAC.
 */
extern "C" void ETH_IRQHandler(void) {
	gNetwork->mac->handleIRQ();
}
