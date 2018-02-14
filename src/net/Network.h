/*
 * Network.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef NET_NETWORK_H_
#define NET_NETWORK_H_

class Network {
	public:
		static void init(void);
		static Network *sharedInstance(void) noexcept;

	private:
		Network();

		void startNetServices(void);

	private:
		virtual ~Network();
};

#endif /* NET_NETWORK_H_ */
