
#ifndef _THOR_H
#define _THOR_H

#include <stdint.h>
#include <avr/pgmspace.h>

typedef enum {
	THOR4,
	THOR5,
	THOR8,
	THOR11,
	THOR16,
	THOR22,
	THOR25x4,
	THOR50x1,
	THOR50x2,
	THOR100,
} thor_mode_t;

extern void thor_init(thor_mode_t mode);
extern void inline thor_wait(void);
extern void thor_data(uint8_t *data, size_t length);
extern void thor_data_P(PGM_P data, size_t length);
extern void thor_string(char *s);
extern void thor_string_P(PGM_P s);

#endif

