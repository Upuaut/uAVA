

#ifndef __CONFIG_H__
#define __CONFIG_H__

// RADIO CONFIG
#define RADIO_FREQUENCY   434447350UL // Generally desired + 1350
//#define RTTY
//#define DOMINOEX

#define CALLSIGN "NULL"

// RTTY CONFIG
#define ASCII 7          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY  0
#define RTTY_BAUD 50     // Baud rate for RTTY 

/* THOR settings (slow, not as slow and non-standard) */
//#define THOR8
#define THOR16
//#define THOR32


// POWER SAVING SETTING
#define POWERSAVING      // Comment out to turn power saving off

// BOARD SETTINGS
#define STATUS_LED 4     // PAVA R9 Boards have an LED on PIN4
#define GPS_ENABLE 3
#define SHUTDOWN_SI406x_PIN   5
#define AUDIO_PIN             6
#define BATTERY_ADC  A0
#define SOLARPANEL_ADC  A1
#define VCXO_FREQ 16369000L // pAVAR9 Crystal = 16.369 Mhz
#define F_CPU 2000000

#endif

