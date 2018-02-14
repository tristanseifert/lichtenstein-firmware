/*
 * Board.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef BOARD_BOARD_H_
#define BOARD_BOARD_H_

#include <stdint.h>

class Board {
	public:
		static void init(void);
		static Board* sharedInstance(void);

	public:
		typedef enum {
			kBoardLEDIdle,
			kBoardLEDIPAct,
			kBoardLEDOutputAct,
			kBoardLEDError
		} board_led_t;

		void setLED(board_led_t led, uint8_t state);

	private:
		Board();
		virtual ~Board();
};

#endif /* BOARD_BOARD_H_ */
