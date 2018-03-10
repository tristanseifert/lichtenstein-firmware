/*
 * Random.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */
#include "Random.h"

#include <cstddef>

// bit rotation macro
#define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))

namespace sys {

/**
 * Initializes the random number generator.
 *
 * @param seed Value to seed the generator with.
 */
Random::Random(uint32_t seed) {
	// initialize context
	this->context.a = 0xf1ea5eed;
	this->context.b = this->context.c = this->context.d = seed;

	// advance the internal state
	for(size_t i = 0; i < Random::initIterations; ++i) {
		(void) this->getRandom();
	}
}

/**
 * This does nothing.
 */
Random::~Random() {

}

/**
 * Advances the random generator and returns the new value.
 *
 * @return Random 32-bit value
 */
uint32_t Random::getRandom(void) {
	uint32_t e = this->context.a - rot(this->context.b, 27);
	this->context.a = this->context.b ^ rot(this->context.c, 17);

	this->context.b = this->context.c + this->context.d;
	this->context.c = this->context.d + e;
	this->context.d = e + this->context.a;

	return this->context.d;
}

}
