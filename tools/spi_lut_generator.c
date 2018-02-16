/**
 * Generates a look-up table for each possible byte value corresponding to the
 * appropriate SPI waveform.
 *
 * Compile with `clang spi_lut_generator.c -o spi_lut_generator`
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/// contains the bit form for a particular bit in string form
char *bitForm[] = {
	"100", // 0 bit
	"110", // 1 bit
};

/// masks for each of the bits in a byte
uint8_t bitMasks[] = {
	0x80, 0x40, 0x20, 0x10,
	0x08, 0x04, 0x02, 0x01
};

int main() {
	printf("static const uint32_t bitPatternLut[] = {\n");

	for(unsigned int i = 0; i <= 0xFF; i++) {
		// printf("\t// byte 0x%02x\n", i);

		printf("\t0b");

		for(int j = 0; j < 8; j++) {
			int index = (i & bitMasks[j]) ? 1 : 0;
			printf("%s", bitForm[index]);
		}

		printf(", // 0x%02x\n", i);
	}

	printf("};\n");
}
