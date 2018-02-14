/*
 * Clock.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef CLOCK_CLOCK_H_
#define CLOCK_CLOCK_H_

class Network;

class Clock {
	public:
		static void init(void);
		static Clock *sharedInstance() noexcept;

	protected:
		friend class Network;

		void startNTPClient(void);

	private:
		Clock();
		void enableRTCHardware();
		void RTCClockConfig();

	private:
		virtual ~Clock();
};

#endif /* CLOCK_CLOCK_H_ */
