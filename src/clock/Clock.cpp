/*
 * Clock.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#define LOG_MODULE "CLK"

#include "Clock.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_device.h"

static Clock *gClock = nullptr;

/**
 * Allocates the shared clock.
 */
void Clock::init(void) {
	taskENTER_CRITICAL();

	if(!gClock) {
		gClock = new Clock();
	}

	taskEXIT_CRITICAL();
}

/**
 * Returns the shared clock instance.
 */
Clock *Clock::sharedInstance() noexcept {
	return gClock;
}

/**
 * Instantiates the clock. This sets up the RTC hardware, as well as starting
 * the NTP task.
 */
Clock::Clock() {
	this->enableRTCHardware();
}

/**
 * Enables the RTC hardware.
 */
void Clock::enableRTCHardware() {
	// enable power to the RTC
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	// set up the clocks for the RTC
	this->RTCClockConfig();
}

/**
 * Sets up the clocks for the RTC.
 */
void Clock::RTCClockConfig() {
	// enable access to config registers
	while(RTC_GetFlagStatus(RTC_FLAG_RTOFF) == RESET) {}
	RTC->CRL |= RTC_CRL_CNF;

#if HW == HW_MUSTARD
	// if we are on the mustard board, we have the LSE oscillator
	RCC_LSEConfig(RCC_LSE_ON);
	// wait for the LSE to be available
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}

	// select it as the clock source
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	// set prescaler
	RTC->PRLH = 0;
	RTC->PRLL = 0x7FFF;
#else
	// otherwise, use the LSI oscillator (ew)
	RCC_LSICmd(ENABLE);

	// wait for the LSI to be available
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}

	// select it as the clock source
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#endif

	// disable access to config registers
	RTC->CRL &= (uint16_t) ~(RTC_CRL_CNF);
//	while(RTC_GetFlagStatus(RTC_FLAG_RTOFF) == RESET) {}
}

/**
 * Performs some teardown, namely stopping the NTP task.
 */
Clock::~Clock() {

}

/**
 * Sets up the NTP client once the network stack has started.
 */
void Clock::startNTPClient(void) {

}

