/*
 * init.h
 *
 *  Created on: Feb 12, 2018
 *      Author: tristan
 */

#ifndef NET_INIT_H_
#define NET_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the network stack. This reads the MAC address from the config
 * memory, and starts the IP stack.
 */
void net_init(void);

#ifdef __cplusplus
}
#endif

#endif /* NET_INIT_H_ */
