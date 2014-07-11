
/* An adaptation of the THOR modulator from fldigi */
/* GPL3+ */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "thor.h"
#include "thor_varicode.h"

/* For Si radio channel setting */
#include "si406x.h"

/* THOR uses 18 tones */
#define TONES 18

/* Interleaver settings */
#define INTER_SIZE 4
#define INTER_LEN (THOR_INTER * INTER_SIZE * INTER_SIZE)
#define INTER_BYTES (INTER_LEN / 8)

/* THOR state */
static uint16_t _conv_sh;
static uint8_t _preamble;
static uint8_t _inter_table[INTER_BYTES];
static uint16_t _inter_offset;

/* Message being sent */
volatile static uint8_t  _txpgm;
volatile static uint8_t *_txbuf;
volatile static uint16_t _txlen;

static inline void _wtab(uint16_t i, uint8_t v)
{
	if(i >= INTER_LEN) i -= INTER_LEN;
	if(!v) _inter_table[i >> 3] &= ~(1 << (7 - (i & 7)));
	else   _inter_table[i >> 3] |= 1 << (7 - (i & 7));
}

/* Calculate the parity of byte x */
static inline uint8_t _parity(uint8_t x)
{
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return(x & 1);
}

static inline uint16_t _thor_lookup_code(uint8_t c, uint8_t sec)
{
	/* Primary character set */
	if(!sec) return(pgm_read_word(&_varicode[c]));
	
	/* Secondary character set (restricted range) */
	if(c >= ' ' && c <= 'z')
		return(pgm_read_word(&_varicode_sec[c - ' ']));
	
	/* Else return NUL */
	return(pgm_read_word(&_varicode[0]));
}



void thor_init(void)
{
	/* Clear the THOR state */
	_conv_sh = 0;
	_txpgm = 0;
	_txbuf = NULL;
	_txlen = 0;
	_preamble = 16;
	
	memset(_inter_table, 0, INTER_BYTES);
	_inter_offset = 0;
	
	/* The modem is driven by TIMER1 in CTC mode */
	TCCR1A = 0; /* Mode 2, CTC */
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10); /* prescaler /1024 */
	OCR1A = F_CPU / 1024 / THOR_BAUDRATE - 1;
	TIMSK1 = _BV(OCIE1A); /* Enable interrupt */
}

void inline thor_wait(void)
{
	/* Wait for interrupt driven TX to finish */
	while(_txlen > 0) while(_txlen > 0);
}

void thor_data(uint8_t *data, size_t length)
{
	thor_wait();
	_txpgm = 0;
	_txbuf = data;
	_txlen = length;
}

void thor_data_P(PGM_P data, size_t length)
{
	thor_wait();
	_txpgm = 1;
	_txbuf = (uint8_t *) data;
	_txlen = length;
}

void thor_string(char *s)
{
	uint16_t length = strlen(s);
	thor_data((uint8_t *) s, length);
}

void thor_string_P(PGM_P s)
{
	uint16_t length = strlen_P(s);
	thor_data_P(s, length);
}

