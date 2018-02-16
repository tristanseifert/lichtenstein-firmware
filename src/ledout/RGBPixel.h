/*
 * RGBPixel.h
 *
 *  Created on: Feb 16, 2018
 *      Author: tristan
 */

#ifndef LEDOUT_RGBPIXEL_H_
#define LEDOUT_RGBPIXEL_H_

#include <stdint.h>

typedef struct __attribute__ ((__packed__)) {
	uint8_t r, g, b;
} rgb_pixel_t;

typedef struct __attribute__ ((__packed__)) {
	uint8_t r, g, b, w;
} rgbw_pixel_t;

#endif /* LEDOUT_RGBPIXEL_H_ */
