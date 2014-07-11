
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <util/delay.h>
#include <util/twi.h>
#include "thor.h"
#include "thor_varicode.h"

#define DAC (0x10) // DAC I2C Address is 16
#define LED_OK 6            // PAVA R7 Boards have an LED on PIN4
#define DOMEXSPACING 0x67 // EX22
#define POWERSAVING      // Comment out to turn power saving off
#define LMT2ENABLE 2
//#define LMT2_CHANNEL 29

//#define SAMPLERATE (F_CPU / 256 / 5)
#define SAMPLERATE (12500)
#define TONES (18)
//SoftwareSerial LMT2_P0(12, A2); // RX, TX
/* Maximum size of the interleaver table */
#define SIZE (4)
#define INTER_MAX (50 * SIZE * SIZE)
unsigned int lmt2dac = 0x4FC0;
/* THOR timing */
volatile static uint16_t _sr;
volatile static uint16_t _sl;

/* Convolutional encoder settings */
volatile static uint8_t _conv_polya;
volatile static uint8_t _conv_polyb;

/* Interleaver settings */
volatile static uint8_t _inter_depth;

/* THOR state */

static uint8_t _conv_sh;
static uint8_t _inter_table[INTER_MAX];

/* Message being sent */
volatile static uint8_t  _txpgm;
volatile static uint8_t *_txbuf;
volatile static uint16_t _txlen;

void setup() {
    TWSR = 0;
  TWBR = ((F_CPU / 100000L) - 16) / 2;
  TWSR = _BV(TWPS1) | _BV(TWPS0); /* /64 prescaler */
  TWBR = 0;
   digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
  	_delay_ms(500);
  pinMode(LMT2ENABLE, OUTPUT);     
  digitalWrite(LMT2ENABLE,HIGH);
	/* Enable interrupts */
	sei();
       //  lmt2_setchan();	
	DDRB |= _BV(1);
	thor_init(THOR11);
}
void loop() {
  	while(1)
	{
		thor_string_P(PSTR("Hello, World!\n"));
		thor_string_P(PSTR("test0 test1 test2 test3 test4 test5 test6 test7 test8 test9\n"));
	}
}

static uint8_t *tab(int i, int j, int k)
{
	return(&_inter_table[(SIZE * SIZE * i) + (SIZE * j) + k]);
}

static void _thor_interleave_symbols(uint8_t *psyms)
{
	uint8_t i, j, k;
	
	for(k = 0; k < _inter_depth; k++)
	{
		for(i = 0; i < SIZE; i++)
			for(j = 0; j < SIZE - 1; j++)
				*tab(k, i, j) = *tab(k, i, j + 1);
		
		for(i = 0; i < SIZE; i++)
			*tab(k, i, SIZE - 1) = psyms[i];
		
		for(i = 0; i < SIZE; i++)
			psyms[i] = *tab(k, i, SIZE - i - 1);
	}
}

static void _thor_interleave(uint8_t *pbits)
{
	uint8_t i, syms[SIZE];
	
	for(i = 0; i < SIZE; i++)
		syms[i] = (*pbits >> (SIZE - i - 1)) & 1;
	
	_thor_interleave_symbols(syms);
	
	for (*pbits = i = 0; i < SIZE; i++)
		*pbits = (*pbits << 1) | syms[i];
}

/* Calculate the parity of byte x */
static inline uint8_t parity(uint8_t x)
{
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return(x & 1);
}

/* Convolutional encoder */
static uint8_t _thor_conv_encode(uint8_t bit)
{
	uint8_t r;
	
	_conv_sh = (_conv_sh << 1) | bit;
	
	r  = parity(_conv_sh & _conv_polya);
	r |= parity(_conv_sh & _conv_polyb) << 1;
	
	return(r);
}

static uint16_t _thor_lookup_code(uint8_t c, uint8_t sec)
{
	/* Primary character set */
	if(!sec) return(pgm_read_word(&_varicode[c]));
	
	/* Secondary character set (restricted range) */
	if(c >= ' ' && c <= 'z')
		return(pgm_read_word(&_varicode_sec[c - ' ']));
	
	/* Else return NUL */
	return(pgm_read_word(&_varicode[0]));
}

ISR(TIMER0_COMPA_vect)
{
	static uint16_t da = 0, db = 0;
	static uint16_t code;
	static uint8_t tone = 0;
	static uint8_t len = 0;
	uint8_t dacbuf[3];
	uint16_t dacval;

	uint8_t i, bit_sh;
	
	/* Timing */
	while(da < _sr)
	{
		db++;
		da += SAMPLERATE;
	}
	da -= _sr;
	
	if(db < _sl) return;
	db = 0;
	
	/* Transmit the tone */
	OCR2A = 0x80 + tone;
	PORTB ^= _BV(5);
        dacval = lmt2dac + (tone * DOMEXSPACING);
	dacbuf[0] = 0x03;
	dacbuf[1] = dacval >> 8;
	dacbuf[2] = dacval & 0xFF;
	_tw_write(DAC, dacbuf, 3);	


	/* Calculate the next tone */
	bit_sh = 0;
	for(i = 0; i < 2; i++)
	{
		uint8_t data;
		
		/* Done sending the current varicode? */
		if(!len)
		{
			if(_txlen)
			{
				/* Read the next character */
				if(_txpgm == 0) data = *(_txbuf++);
				else data = pgm_read_byte(_txbuf++);
				_txlen--;
			}
			else data = 0;
			
			/* Get the varicode for this character */
			code = _thor_lookup_code(data, 0);
			len  = code >> 12;
		}
		
		/* Feed the next bit into the convolutional encoder */
		data = _thor_conv_encode((code >> --len) & 1);
		
		bit_sh = (bit_sh << 1) | (data & 1);
		bit_sh = (bit_sh << 1) | ((data >> 1) & 1);
	}
	
	_thor_interleave(&bit_sh);
	tone = (tone + 2 + bit_sh) % TONES;
}

void thor_init(thor_mode_t mode)
{
	/* Clear the THOR state */
	_conv_sh = 0;
	_txpgm = 0;
	_txbuf = NULL;
	_txlen = 0;
	
	/* Default settings (THOR11) */
	_sr = 11025;
	_sl = 1024;
	_conv_polya = 0x6D;
	_conv_polyb = 0x4F;
	_inter_depth = 10;
	
	/* TODO */
	switch(mode)
	{
	case THOR4:    break;
	case THOR5:    break;
	case THOR8:    break;
	case THOR11:   break;
	case THOR16:   break;
	case THOR22:   break;
	case THOR25x4: break;
	case THOR50x1: break;
	case THOR50x2: break;
	case THOR100:  break;
	default: return;
	}
	
	memset(_inter_table, 0, _inter_depth * SIZE * SIZE);
	
	/* Fast PWM mode, non-inverting output on OC2A */
	TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	OCR2A = 0x80;
	
	/* The modem is driven by TIMER0 in CTC mode */
	TCCR0A = _BV(WGM01); /* Mode 2, CTC */
	TCCR0B = _BV(CS02); /* prescaler 256 */
	OCR0A = F_CPU / 256 / SAMPLERATE - 1;
	TIMSK0 = _BV(OCIE0A); /* Enable interrupt */
	
	/* Testing on the LED pin */
	DDRB |= _BV(DDB5) | _BV(DDB3);
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

static uint8_t _tw_write(uint8_t addr, uint8_t *ptr, uint8_t len)
{
  addr <<= 1;
  uint8_t r;
  /* Send Start Condition */
  r = _tw_send(_BV(TWSTA));
  if(r != TW_START && r != TW_REP_START) return(r);
  /* Send the address to begin writing at */
  r = _tw_tx(addr);
  if(r != TW_MT_SLA_ACK) return(r);
  /* Transmit the bytes */
  while(len--)
  {
    r = _tw_tx(*(ptr++));
    if(r != TW_MT_DATA_ACK) return(r);
  }

  /* Send Stop Condition */
  _tw_stop();

  return(1);
}

static uint8_t _tw_send(uint8_t flag)
{
  /* Set the flags, wait for completion and return result */
  uint8_t i;

  TWCR = _BV(TWINT) | _BV(TWEN) | flag;
  for(i = 0; i < 200; i++)
  {
    if(TWCR & _BV(TWINT)) return(TW_STATUS);
    _delay_us(1);
  }

  return(0);
}

static uint8_t _tw_tx(uint8_t b)
{
  TWDR = b;
  return(_tw_send(0));
}

static void _tw_stop()
{
  uint8_t i;

  /* Send Stop Condition */
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
  for(i = 0; i < 200 && (TWCR & _BV(TWSTO)); i++)
    _delay_us(1);
}



