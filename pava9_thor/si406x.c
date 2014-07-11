
/* Ported to C by Philip Heron <phil@sanslogic.co.uk>                    */
/* Code modified by Arko for the Si406x                                  */
/* Code based on KT5TK's Si446x                                          */
/* pecanpico2 Si446x Driver copyright (C) 2013  KT5TK                    */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "config.h"
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "si406x.h"

/* Shutdown pin for the radio is PD5 (arduino pin 5) */
#define SDN(b) PORTD = (PORTD & (~_BV(5))) | ((b) ? _BV(5) : 0)

/* SS pin */
#define SS(b) PORTB = (PORTB & (~_BV(2))) | ((b) ? _BV(2) : 0)

/* Commands */
#define CMD_POWER_UP     0x02
#define CMD_SET_PROPERTY 0x11
#define CMD_START_TX     0x31
#define CMD_CHANGE_STATE 0x34
#define CMD_GET_ADC_READING  0x14

/* States */
#define STATE_READY   0x03
#define STATE_TX_TUNE 0x05
#define STATE_TX      0x07

/* Receive buffer */
static uint8_t _rxbuf[8];

static inline uint8_t _spi_transfer(uint8_t v)
{
  /* TODO: There needs to be a timeout here */
  SPDR = v;
  while(!(SPSR & _BV(SPIF)));
  return(SPDR);
}



static void _wait_for_cts()
{
  uint8_t r;

  /* GPIO1 pin method */
  /* TODO: Add a timeout here */
  /*while(PIND & _BV(6));*/

  /* Poll CTS over SPI method */
  do
  {
    SS(0);

    _spi_transfer(0x44);
    r = _spi_transfer(0x00);

    SS(1);
  }
  while(r != 0xFF);
}
static void _wait_for_response(uint8_t *buf, uint8_t len)
{
  uint8_t r;
  unsigned long startTime = millis();
  /* Poll CTS over SPI ABCD */
  while(((millis() - startTime) < 1000))
  {
    SS(0);
    
    _spi_transfer(0x44);
    r = _spi_transfer(0xFF);
    
    if(r == 0xFF) break;
    
    SS(1);
    
    _delay_us(10);
  }
  
  /* Read the requested data */
  while(len--) *(buf++) = _spi_transfer(0xFF);
  
  SS(1);
}
static void _send_command(uint8_t cmd, uint8_t *data, uint8_t length)
{
  _wait_for_cts();

  /* Set SS low to select chip */
  SS(0);
  _delay_us(1); /* Output enable time, 20ns */

  /* Send the command and data */
  _spi_transfer(cmd);
  for(; length; length--)
    _spi_transfer(*(data++));

  /* Unselect the chip */
  SS(1);
}

static void _power_up()
{
  uint8_t data[] = {
    0x01, 0x01, /* No patch, boot main app. img, TXCO */
    (VCXO_FREQ >> 24) & 0xFF, /* VCXO frequency */
    (VCXO_FREQ >> 16) & 0xFF,
    (VCXO_FREQ >> 8) & 0xFF,
    VCXO_FREQ & 0xFF
  };

  _send_command(CMD_POWER_UP, data, sizeof(data));
}

static void _set_modem()
{
  /* Set to 2FSK mode, driven from GPIO1 pin */
  uint8_t data[] = { 
    0x20, 0x01, 0x00, 0xAA     };
  _send_command(CMD_SET_PROPERTY, data, sizeof(data));
}

static void _set_state(uint8_t state)
{
  _send_command(CMD_CHANGE_STATE, &state, 1);
}

void si_init(void)
{
  /* Set the SDN pin for output */
  DDRD |= _BV(5);

  /* Set the CTS pin for input */
  DDRD &= ~_BV(6);

  /* Shutdown the radio */
  SDN(1);

  /* Configure SPI pins for output */
  DDRB |= _BV(2) | _BV(3) | _BV(5); /* PB2:SS PB3:MOSI PB5:SCK */
  SS(1);                             /* SS high */

  /* Configure SPI input pin PB4:MISO, enable internal pull-up */
  DDRB  &= ~_BV(4);
  PORTB |= _BV(4);

  /* Enable SPI hardware, Master mode, MSB first, F_CPU / 4 */
  SPCR  = _BV(SPE) | _BV(MSTR);
  SPSR &= _BV(SPI2X);

  /* "The SDN pin needs to be held high for
   	 *  at least 10us before driving low again
   	 *  so that internal capacitors can discharge" */
  _delay_us(20);

  /* Power up the radio */
  SDN(0);

  /* "High time for VDD to fully settle POR circuit." (Min: 10ms) */
  _delay_ms(20);

  /* Send the POWER_UP command */
  _power_up();

  /* Set the initial frequency */
  //	si_set_frequency(434501000UL);
  si_set_frequency(RADIO_FREQUENCY);

  /* Set to CW mode */
  _set_modem();

  /* Set the "TX Tune" state */
  _set_state(STATE_TX_TUNE);
}

void si_radio_on(void)
{
  //_set_state(STATE_TX_TUNE);
  _set_state(STATE_TX);
}

void si_radio_off(void)
{
  _set_state(STATE_READY);
}

void si_set_frequency(uint32_t freq)
{
  uint8_t outdiv, band;
  uint32_t f_pfd, n, m;
  float ratio, rest;

  /* Set the output divider according to the recommended ranges in the si406x datasheet */
  if(freq < 177000000UL)      { 
    outdiv = 24; 
    band = 5; 
  }
  else if(freq < 239000000UL) { 
    outdiv = 16; 
    band = 4; 
  }
  else if(freq < 353000000UL) { 
    outdiv = 12; 
    band = 3; 
  }
  else if(freq < 525000000UL) { 
    outdiv = 8;  
    band = 2; 
  }
  else if(freq < 705000000UL) { 
    outdiv = 6;  
    band = 1; 
  }
  else                        { 
    outdiv = 4;  
    band = 0; 
  }

  f_pfd = 2 * VCXO_FREQ / outdiv;
  n = freq / f_pfd - 1;

  ratio = (float) freq / f_pfd;
  rest  = ratio - n;

  m = rest * 524288UL;

  /* set the band parameter */
  {
    uint8_t data[] = { 
      0x20, 0x01, 0x51, 8 + band         };
    _send_command(CMD_SET_PROPERTY, data, sizeof(data));
  }

  /* Set the pll parameters */
  {
    uint8_t data[] = {
      0x40, 0x06, 0x00, n,
      (m >> 16) & 0xFF,
      (m >> 8) & 0xFF,
      m & 0xFF,
      0x00, 0x02
    };
    _send_command(CMD_SET_PROPERTY, data, sizeof(data));
  }

  /* Set frequency deviation */
  {
    uint8_t data[] = {
      0x20, 0x03, 0x0A, 0x00, 0x00, 0x22 /* 0x22 sets RTTY shift at about 450 */
    };
    _send_command(CMD_SET_PROPERTY, data, sizeof(data));
  }
}

void si_set_channel(uint8_t channel)
{
  uint8_t data[] = { 
    channel, 0x23, 0x00, 0x00     };

  /* Change to ready state */
  _set_state(STATE_READY);

  /* Send START_TX command */
  _send_command(CMD_START_TX, data, sizeof(data));
}

int16_t si_get_temperature()
{
  uint8_t data[] = { 0x10, 0x00 };
  int32_t temp;
  
  _send_command(CMD_GET_ADC_READING, data, sizeof(data));
  
  _wait_for_response(_rxbuf, 8);
  
  /* Calculate the temperature in C * 10 */
  temp  = (_rxbuf[4] << 8) | _rxbuf[5];
  temp *= 568;
  temp /= 256;
  temp -= 2970;
  
  return(temp);
}
