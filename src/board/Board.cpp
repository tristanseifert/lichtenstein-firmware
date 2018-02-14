/*
 * Board.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */
#include "Board.h"

#include "FreeRTOSConfig.h"
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

// TEST_SW0
#define TEST_SW0_PORT				GPIOC
#define TEST_SW0_PIN					GPIO_Pin_13
#define TEST_SW0_GPIO_CLOCK			RCC_APB2Periph_GPIOC

#define TEST_SW0_GPIO_EXTI			EXTI15_10_IRQHandler
#define TEST_SW0_GPIO_EXTI_PORT		GPIO_PortSourceGPIOC
#define TEST_SW0_GPIO_EXTI_PIN		GPIO_PinSource13
#define TEST_SW0_GPIO_EXTI_LINE		EXTI_Line13
#define TEST_SW0_GPIO_EXTI_NVIC_IRQ	EXTI15_10_IRQn
#define TEST_SW0_GPIO_EXTI_PRIORITY	(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// TEST_SW1
#define TEST_SW1_PORT				GPIOB
#define TEST_SW1_PIN					GPIO_Pin_2
#define TEST_SW1_GPIO_CLOCK			RCC_APB2Periph_GPIOB

#define TEST_SW1_GPIO_EXTI			EXTI2_IRQHandler
#define TEST_SW1_GPIO_EXTI_PORT		GPIO_PortSourceGPIOB
#define TEST_SW1_GPIO_EXTI_PIN		GPIO_PinSource2
#define TEST_SW1_GPIO_EXTI_LINE		EXTI_Line2
#define TEST_SW1_GPIO_EXTI_NVIC_IRQ	EXTI2_IRQn
#define TEST_SW1_GPIO_EXTI_PRIORITY	(configMAX_SYSCALL_INTERRUPT_PRIORITY >> 4)

// when set, the EXTI is shared among both test switch inputs
#define TEST_SW_GPIO_EXTI_SHARED		0
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
	this->initStatusGPIOs();
	this->initTestGPIOs();
}
Board::~Board() {
	// TODO Auto-generated destructor stub
}



/**
 * Initializes the GPIO pins for the status LEDs.
 */
void Board::initStatusGPIOs(void) {
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

	// set default state for LEDs
	this->setLED(kBoardLEDIdle, 0);
	this->setLED(kBoardLEDIPAct, 0);
	this->setLED(kBoardLEDOutputAct, 0);
	this->setLED(kBoardLEDError, 0);
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
 * Initializes the GPIOs used for entering various test modes via a DIP switch.
 * These inputs will be at 3V3 when the associated bit is set, or weakly pulled
 * down otherwise.
 */
void Board::initTestGPIOs(void) {
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;


	// set up TEST_SW0
	RCC_APB2PeriphClockCmd(TEST_SW0_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = TEST_SW0_PIN;
	GPIO_Init(TEST_SW0_PORT, &gpio);

	// set up TEST_SW1
	RCC_APB2PeriphClockCmd(TEST_SW1_GPIO_CLOCK, ENABLE);

	gpio.GPIO_Pin = TEST_SW1_PIN;
	GPIO_Init(TEST_SW1_PORT, &gpio);


	// enable the external interrupts from those pins
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nvic;
	EXTI_StructInit(&exti);

	GPIO_EXTILineConfig(TEST_SW0_GPIO_EXTI_PORT, TEST_SW0_GPIO_EXTI_PIN);
	GPIO_EXTILineConfig(TEST_SW1_GPIO_EXTI_PORT, TEST_SW1_GPIO_EXTI_PIN);

	// configure the SW0 exti and then enable the interrupt
	exti.EXTI_Line = TEST_SW0_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = TEST_SW0_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW0_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	// configure the SW1 exti and then enable the interrupt
	exti.EXTI_Line = TEST_SW1_GPIO_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	nvic.NVIC_IRQChannel = TEST_SW1_GPIO_EXTI_NVIC_IRQ;
	nvic.NVIC_IRQChannelPreemptionPriority = TEST_SW1_GPIO_EXTI_PRIORITY;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

/**
 * Called from the IRQ handler for the test switches. This will read out the
 * value of both switches, and notify other processes.
 */
void Board::testSwIRQ(void) {
	bool sw0 = (GPIO_ReadInputDataBit(TEST_SW0_PORT, TEST_SW0_PIN) == Bit_SET);
	bool sw1 = (GPIO_ReadInputDataBit(TEST_SW1_PORT, TEST_SW1_PIN) == Bit_SET);

	this->testSwState = (uint8_t) ((sw0 ? 0x01 : 0x00) | (sw1 ? 0x02 : 0x00));

//	this->setLED(kBoardLEDIPAct, sw0);
//	this->setLED(kBoardLEDOutputAct, sw1);
}

/**
 * Handles the IRQ for the TEST_SW0 external interrupt.
 */
extern "C" void TEST_SW0_GPIO_EXTI(void) {
	Board::TestSwIRQNotify();
}

/**
 * Handles the IRQ for the TEST_SW1 external interrupt, if it is different from
 * the one used by the first switch.
 */
#if !TEST_SW_GPIO_EXTI_SHARED
extern "C" void TEST_SW1_GPIO_EXTI(void) {
	Board::TestSwIRQNotify();
}
#endif

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

