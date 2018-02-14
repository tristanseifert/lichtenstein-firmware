/*
 * Board.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Board.h"

#include "cmsis_device.h"

#if HW == HW_MUSTARD
// STATUS0 LED (idle)
#define STATUS0_PORT GPIOD
#define STATUS0_PIN GPIO_Pin_2
#define STATUS0_GPIO_CLOCK RCC_APB2Periph_GPIOD

// STATUS1 LED (IP act)
#define STATUS1_PORT GPIOD
#define STATUS1_PIN GPIO_Pin_3
#define STATUS1_GPIO_CLOCK RCC_APB2Periph_GPIOD

// STATUS2 LED (LED output on)
#define STATUS2_PORT GPIOD
#define STATUS2_PIN GPIO_Pin_4
#define STATUS2_GPIO_CLOCK RCC_APB2Periph_GPIOD

// STATUS3 LED (Error)
#define STATUS3_PORT GPIOD
#define STATUS3_PIN GPIO_Pin_7
#define STATUS3_GPIO_CLOCK RCC_APB2Periph_GPIOD
#endif


static Board *gBoard = nullptr;


/**
 * Initializes the shared board instance.
 */
void Board::init() {
	if(!gBoard) {
		gBoard = new Board();
	}
}

/**
 * Returns the shared instance of the board.
 */
Board *Board::sharedInstance() {
	return gBoard;
}

/**
 * Initializes the board.
 */
Board::Board() {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;

	// set up STATUS0
	RCC_APB2PeriphClockCmd(STATUS0_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS0_PIN;
	GPIO_Init(STATUS0_PORT, &gpio);
	// set up STATUS1
	RCC_APB2PeriphClockCmd(STATUS1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS1_PIN;
	GPIO_Init(STATUS1_PORT, &gpio);
	// set up STATUS2
	RCC_APB2PeriphClockCmd(STATUS2_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS2_PIN;
	GPIO_Init(STATUS2_PORT, &gpio);
	// set up STATUS3
	RCC_APB2PeriphClockCmd(STATUS3_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = STATUS3_PIN;
	GPIO_Init(STATUS3_PORT, &gpio);

	this->setLED(kBoardLEDIdle, 0);
	this->setLED(kBoardLEDIPAct, 0);
	this->setLED(kBoardLEDOutputAct, 0);
	this->setLED(kBoardLEDError, 0);
}

Board::~Board() {
	// TODO Auto-generated destructor stub
}

/**
 * Sets the state of the status indicators. Each LED on the board has an index:
 * - 0: idle
 * - 1: IP Act
 * - 2: LED out active
 * - 3: Error
 */
void Board::setLED(board_led_t led, uint8_t state) {
	if(led == kBoardLEDIdle) {
		GPIO_WriteBit(STATUS0_PORT, STATUS0_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDIPAct) {
		GPIO_WriteBit(STATUS1_PORT, STATUS1_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDOutputAct) {
		GPIO_WriteBit(STATUS2_PORT, STATUS2_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	} else if(led == kBoardLEDError) {
		GPIO_WriteBit(STATUS3_PORT, STATUS3_PIN, (state == 1 ? Bit_SET : Bit_RESET));
	}
}


/**
 * Set the idle LED in the idle task hook.
 */
extern "C" void vApplicationIdleHook(void) {
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, 1);
}

/**
 * Clear the idle LED in the tick hook.
 */
extern "C" void vApplicationTickHook(void) {
	Board::sharedInstance()->setLED(Board::kBoardLEDIdle, 0);
}

