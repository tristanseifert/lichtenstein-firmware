/*
 * Errors.h
 *
 *  Created on: Feb 13, 2018
 *      Author: tristan
 */

#ifndef INCLUDE_ERRORS_H_
#define INCLUDE_ERRORS_H_

#include <stdexcept>
#include <exception>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

class TaskCreationError : public std::runtime_error {
	public:
		TaskCreationError(BaseType_t errorCode) : std::runtime_error("TaskCreationError"), status(errorCode) {}

		BaseType_t getStatus() const {
			return this->status;
		}

	private:
		BaseType_t status;
};


#endif /* INCLUDE_ERRORS_H_ */
