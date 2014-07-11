
#ifndef _THOR_H
#define _THOR_H

/* Both THOR8 and THOR16 use 15.625 Hz frequency steps */

#if defined(THOR8)
  #define THOR_BAUDRATE 7.8125
  #define THOR_DS 1
#elif defined(THOR16)
  #define THOR_BAUDRATE 15.625
  #define THOR_DS 1
#elif defined(THOR32)
  #define THOR_BAUDRATE 31.25
  #define THOR_DS 2
#endif

#define THOR_K 7
#define THOR_POLYA 0x6D
#define THOR_POLYB 0x4F
#define THOR_INTER 10

#include <stdint.h>
#include <avr/pgmspace.h>

extern void thor_init(void);
extern void inline thor_wait(void);
extern void thor_data(uint8_t *data, size_t length);
extern void thor_data_P(PGM_P data, size_t length);
extern void thor_string(char *s);
extern void thor_string_P(PGM_P s);

#endif

