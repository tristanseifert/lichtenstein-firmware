/*
 * Network.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "NET"

#include "Network.h"
#include "EthMAC.h"

#include "../board/Board.h"
#include "../clock/Clock.h"

#include "LichtensteinApp.h"

#include "FreeRTOS_IP.h"

#include <cstring>

// hardware config for EEPROM
#if HW == HW_MUSTARD

// use the RMII interface
#define ETH_RMII							1

#endif


/// default IP address, if DHCP fails
static const uint8_t ucIPAddress[4] = { 192, 168, 1, 200 };
/// default netmask, if DHCP fails
static const uint8_t ucNetMask[4] = { 255, 255, 255, 0 };
/// default router address, if DHCP fails
static const uint8_t ucGatewayAddress[4] = { 192, 168, 1, 1 };
/// default DNS server address
static const uint8_t ucDNSServerAddress[4] = { 208, 67, 222, 222 };

/// MAC Address, we need to store this again here
uint8_t ucMACAddress[6];

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

	// configure PLL for clock output to PHY
	this->setUpClocks();

	// set up the MAC and PHY, respectively
	this->setUpMAC();
	this->scanForPHYs();

	// set up the stack
	this->setUpStack();

	// when the stack is set up, start network services
	this->startNetServices();
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
	delete this->mac;
}


/**
 * Reads the MAC address from the EEPROM.
 */
void Network::readMACFromEEPROM(void) {
	Board *b = Board::sharedInstance();
	b->configEEPROMRead(&this->macAddress, Network::ethParamMACOffset, 6);

	// copy MAC address
	memcpy(ucMACAddress, this->macAddress, 6);

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
	// get the frequencies of clocks
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);

	// configure PLL3 for 50MHz
	RCC_PLL3Config(RCC_PLL3Mul_10);

	// enable PLL3
	RCC_PLL3Cmd(ENABLE);

	// wait for lock
	int timeout = 20000;

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
}

/**
 * Sets up the clocks for the Ethernet MAC and the various GPIOs used for the
 * RMII interface.
 */
void Network::setUpMAC(void) {
	this->setUpEthernetGPIOs();

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

	// all pins below are 50MHz, AF-PP
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;

	// RMII_REF_CLK (PA1)
	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &gpio);

	// RMII_MDIO (PA2)
	gpio.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &gpio);

	// MCO (PA8)
	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &gpio);

	// RMII_TX_EN (PB11)
	gpio.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &gpio);

	// RMII_TXD[0,1] (PB12, PB13)
	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init(GPIOB, &gpio);

	// RMII_MDC (PC1)
	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOC, &gpio);

	// RMII_RXD[0,1] (PD9, PD10) remap
	GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOD, &gpio);

	// RMII_CRS_DV (PD8) remap
	GPIO_PinRemapConfig(GPIO_Remap_ETH, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOD, &gpio);
}

/**
 * Scans the MDIO bus for any attached PHYs.
 */
void Network::scanForPHYs(void) {
	uint32_t id;

	// scan all PHYs sequentially
	for(uint16_t phy = 0; phy <= 0x1f; phy++) {
		// read its id
		id = this->readPHYId(phy);

		// if the register is mostly 1's, there's no PHY here
		if((id & 0x0000FFFF) == 0x0000FFFF) {
//			LOG(S_VERBOSE, "no phy at %d", phy);
		} else {
			LOG(S_INFO, "Found PHY at %d: 0x%08x", phy, id);
		}
	}

}

/**
 * Reads the 32-bit composite PHY ID register.
 */
uint32_t Network::readPHYId(uint16_t phy) {
	uint32_t reg = 0;
	int temp;

	// read the top half of the register (0x02)
	temp = this->mac->mdioRead(phy, 0x02);

	if(temp < 0) {
		LOG(S_ERROR, "Couldn't read PHYSID1 from %d", phy);
		return 0xFFFFFFFF;
	}

	reg |= ((temp & 0x0000FFFF) << 16);

	// read the lower half of the register (0x03)
	temp = this->mac->mdioRead(phy, 0x03);

	if(temp < 0) {
		LOG(S_ERROR, "Couldn't read PHYSID2 from %d", phy);
		return 0xFFFFFFFF;
	}

	reg |= (temp & 0x0000FFFF);

	// we're done, return the register
	return reg;
}

/**
 * Determines the type of PHY attached and initializes it.
 */
void Network::setUpPHY(uint16_t address) {
	LOG(S_INFO, "Initializing PHY %d", address);

	// TODO: make this do stuff
}


/**
 * Sets up the TCP/IP stack.
 */
void Network::setUpStack(void) {
	// initialize IP stack
//	FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, this->macAddress);
}


/**
 * Generates a random number.
 */
extern "C" UBaseType_t uxRand() {
	static UBaseType_t g_seed = 0xDEADBEEF;

	g_seed = (214013*g_seed+2531011);
	return (g_seed>>16)&0x7FFF;
}

/**
 * Network event hook
 */
extern "C" void vApplicationIPNetworkEventHook(eIPCallbackEvent_t event) {
	if(event == eNetworkUp) {
		LOG(S_INFO, "link up");
	} else if(event == eNetworkDown) {
		LOG(S_INFO, "link down");
	}
}
