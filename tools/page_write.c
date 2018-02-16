/**
 * Testing the page write algorithm for the EEPROM.
 *
 * Compile with `clang page_write.c -o page_write`
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

const uint8_t pageSize = 0x08;
const uint8_t pageSizeMask = 0x07;

void write(uint8_t address, size_t numBytes) {
	int bytesWritten = 0;

	// sanity checking
	if(/*buf == nullptr || */numBytes > 0xFF) {
		return;
	}

	printf("** starting write to 0x%02x, size 0x%02zx\n", address, numBytes);

	// write the first page
	uint8_t firstPage = (address & ~pageSizeMask);
	uint8_t offsetIntoPage = address - firstPage;
	uint8_t bytesToWrite = pageSize - offsetIntoPage;

	uint8_t writeAddress = address;

	if(numBytes < bytesToWrite) {
		bytesToWrite = numBytes;
	}

	printf("\tfirst page is 0x%02x, writing to offset 0x%02x, address 0x%02x, (%i bytes)\n\t", firstPage, offsetIntoPage, writeAddress, bytesToWrite);

	for(int i = 0; i < bytesToWrite; i++) {
		printf("#");
		bytesWritten++; writeAddress++;
	}

	printf("\n\tsending stop condition\n");

	// have we got any more bytes to write?
	if(bytesWritten < numBytes) {
		int bytesLeftToWrite = numBytes - bytesWritten;

		printf("\tremaining bytes to write: %i, starting at 0x%02x\n", bytesLeftToWrite, writeAddress);

		for(int i = 0, j = bytesLeftToWrite; i < j; i++) {
			if((writeAddress & pageSizeMask) == 0) {
				printf("\tstarting page write at 0x%02x for %i bytes\n\t", writeAddress, bytesLeftToWrite & pageSizeMask);

				// send address
			}

			// write byte here lol
			printf("*");
			bytesWritten++; writeAddress++;

			// have we reached the end of the page?
			if((writeAddress & pageSizeMask) == 0) {
				printf("\n\tsending stop condition after writing %i bytes, have %i bytes left\n", pageSize, numBytes - bytesWritten);
			}
		}

		// send a stop condition again in case we haven't already
		printf("\n\tsending stop condition\n");
	}

	printf("\n\n");
}

int main() {
	write(0x00, 4);
	write(0x04, 4);
	write(0x07, 4);
	write(0x08, 16);
	write(0x08, 18);
	write(0x04, 22);
}
