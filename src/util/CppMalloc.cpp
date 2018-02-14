/*
 * CppMalloc.cpp
 *
 * Replaces the standard compiler-provided new/delete operators with ones that
 * instead call into the FreeRTOS memory manager, so that these calls will use
 * the same heap as FreeRTOS.
 *
 *  Created on: Feb 14, 2018
 *      Author: tristan
 */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

#include <cstdint>

/**
 * Override the new operator to call into the FreeRTOS memory manager.
 */
void *operator new(size_t size) {
	void *p = pvPortMalloc(size);

	// if exceptions are enabled, throw an error
#ifdef	__EXCEPTIONS
	if(p == nullptr) {
		throw std::bad_alloc();
	}
#endif

	return p;
}

/**
 * Override the new[] operator to call into the FreeRTOS memory manager.
 */
void *operator new[](size_t size) {
	void *p = pvPortMalloc(size);

	// if exceptions are enabled, throw an error
#ifdef	__EXCEPTIONS
	if(p == nullptr) {
		throw std::bad_alloc();
	}
#endif

	return p;
}

/**
 * Override the delete operator to call into the FreeRTOS memory manager.
 */
void operator delete(void *p) {
	vPortFree(p);
}


