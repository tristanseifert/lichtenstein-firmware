/*
 * Random.h
 *
 * A very simple (not cryptographically secure) random number generator. Copied
 * from http://burtleburtle.net/bob/rand/smallprng.html.
 *
 *  Created on: Mar 9, 2018
 *      Author: tristan
 */
#ifndef SYS_RANDOM_H_
#define SYS_RANDOM_H_

#include <cstddef>

#include <LichtensteinApp.h>



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

namespace sys {
	class Random {
		public:
			Random(uint32_t seed = 0xDEADBEEF);
			virtual ~Random();

		public:
			uint32_t getRandom(void);

		private:
			// how many times to call getRandom on init
			static const size_t initIterations = 20;

			// generator state
			struct {
				uint32_t a, b, c, d;
			} context;
	};
}

#pragma GCC diagnostic pop

#endif /* SYS_RANDOM_H_ */
