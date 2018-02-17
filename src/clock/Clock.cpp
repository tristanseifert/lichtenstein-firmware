/*
 * Clock.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);

	// set up the clocks for the RTC
	this->RTCClockConfig();
}

/**
 * Sets up the clocks for the RTC.
 */
void Clock::RTCClockConfig() {
	// if we are on the mustard board, we have the LSE oscillator
	if(HW == HW_MUSTARD) {

	}
	// otherwise, use the LSI oscillator (ew)
	else {
		RCC_LSICmd(ENABLE);

		// wait for the LSI to be available
		while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {}

		// select it as the clock source
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	}
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

