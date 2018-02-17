/*
 * Network.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "NET"

#include "Network.h"

#include "../board/Board.h"
#include "../clock/Clock.h"

#include "LichtensteinApp.h"

#include "FreeRTOS_IP.h"

#include <cstring>

// hardware config for EEPROM
#if HW == HW_MUSTARD

#endif


/// default IP address, if DHCP fails
static const uint8_t ucIPAddress[ 4 ] = { 192, 168, 1, 200 };
/// default netmask, if DHCP fails
static const uint8_t ucNetMask[ 4 ] = { 255, 255, 255, 0 };
/// default router address, if DHCP fails
static const uint8_t ucGatewayAddress[ 4 ] = { 192, 168, 1, 1 };
/// default DNS server address
static const uint8_t ucDNSServerAddress[ 4 ] = { 208, 67, 222, 222 };

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
	this->setUpEthParamEEPROM();

	// set up the MAC and PHY, respectively
	this->setUpMAC();
	this->setUpPHY();

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

}


/**
 * Sets up the I2C peripheral that's connected to the EEPROM that contains the
 * Ethernet MAC address. This will typically be a 24AA025E48 or similar, with
 * the 6-byte MAC at address $80.
 *
 * This doesn't _have_ to be one of those types of EEPROMs that come from the
 * factory programmed with a MAC, but it's the most convenient option since
 * we don't implement writing to the EEPROM.
 */
void Network::setUpEthParamEEPROM(void) {


    // try to probe for the EEPROM and read the MAC
//    this->_i2cScan();
    this->probeEthParamEEPROM();
}

/**
 * Scans the I2C bus to locate the address of the parameter EEPROM.
 */
void Network::probeEthParamEEPROM(void) {
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
 * Sets up the clocks for the Ethernet MAC and the various GPIOs used for the
 * RMII interface.
 */
void Network::setUpMAC(void) {
	GPIO_InitTypeDef gpio;

	// enable clock for the  DMA engines
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA1, ENABLE);

	// enable clocks for the ETH peripherals
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC_Tx | RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

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
 * Initializes the correct PHY based on the hardware.
 */
void Network::setUpPHY(void) {

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
