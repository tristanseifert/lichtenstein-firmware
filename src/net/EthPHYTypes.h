/*
 * EthPHYTypes.h
 *
 *  Created on: Feb 20, 2018
 *      Author: tristan
 */

#ifndef NET_ETHPHYTYPES_H_
#define NET_ETHPHYTYPES_H_

/**
 * Defines possible values for link speeds.
 */
typedef enum {
	kLinkSpeedUnknown,

	kLinkSpeed10Mbps		= 10,
	kLinkSpeed100Mbps	= 100
} net_link_speed_t;



#endif /* NET_ETHPHYTYPES_H_ */
