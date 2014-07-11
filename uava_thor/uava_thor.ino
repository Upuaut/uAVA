/*
 PAVA R9 Tracker Code
 SI4060 Based RF Module
 
 By Anthony Stirk M0UPU 
 
 December 2013
 Subversion 1.06
 
 Thanks and credits :
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 
 SI4060 Code Code modified by Ara Kourchians for the Si406x originally based on
 KT5TK's Si446x code. 
 
 Code snippets from Project Swift Nigel Smart / Philip Heron
 Big thanks to Dave Akerman, Phil Heron, Mark Jessop, Leo Bodnar for suggestions
 ideas and assistance. 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */

#include <util/twi.h>
#include <util/delay.h>
#include <util/crc16.h>
//#include <SPI.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "thor.h"
#include "thor.c"

#define DAC (0x10) // DAC I2C Address is 16
#define RADIO_ENABLE 2
#define LED_WARN 5
#define LED_OK 6
#define BATTERY_ADC A0
#define POWERSAVING
SoftwareSerial MTX2_EN(12, RADIO_ENABLE); // RX, TX
#define DOMEXSPACING 0x96 // EX22 OK
unsigned int radio2dac = 0x4FC0;
unsigned int dom = 0x0000;
#define ONE_SECOND F_CPU / 1024 / 8
uint8_t buf[60]; 
static char _txstring[80];
static volatile char _txstatus = 0;
volatile int _txstringlength = 0;
volatile char txc;
volatile int txi;
volatile int txj;
uint32_t count=1;
volatile boolean lockvariables = 0;
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
int16_t temp;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int battv=0, navmode = 0, GPSerror = 0, lat_int=0,lon_int=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0;
int psm_status = 0, batteryadc_v=0, battvaverage=0;
int32_t tslf=0;
int32_t battvsmooth[5] ;
int errorstatus=8; 
unsigned long currentMillis;
long previousMillis = 0;

/* 
 Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = No used on pAVA
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 */


void setup() {
  TWSR = 0;
  TWBR = ((F_CPU / 100000L) - 16) / 2;
  TWSR = _BV(TWPS1) | _BV(TWPS0); /* /64 prescaler */
  TWBR = 0;
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
  Serial.begin(9600);
  pinMode(RADIO_ENABLE, OUTPUT);     
  digitalWrite(RADIO_ENABLE,HIGH);
  pinMode(LED_WARN, OUTPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(BATTERY_ADC, INPUT);
  setMTX2Frequency();
  resetGPS();
  setupGPS();
  initialise_interrupt();
  thor_init();
  thor_data_P(PSTR("\0\x0D\x02\x0D"), 4);
  blinkled(1);

}

void loop()
{

  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();

  if(lock!=3) // Blink LED to indicate no lock
  {
    digitalWrite(STATUS_LED, HIGH);   
    wait(750);               
    digitalWrite(STATUS_LED, LOW); 
    errorstatus |=(1 << 5);     
  }
  else
  {
    errorstatus &= ~(1 << 5);
  }
  checkDynamicModel();
#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))
  {
    setGPS_PowerSaveMode();
    wait(1000);
    pinMode(STATUS_LED, INPUT); 
    psm_status=1;
    errorstatus &= ~(1 << 4);
  }
#endif
  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }
  }
  if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=4) {
    tslf++;
  }
  else
  {
    tslf=0;
    errorstatus &= ~(1 << 0);
    errorstatus &= ~(1 << 1);
  }
  if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
    setupGPS();
    wait(125);
    setGps_MaxPerformanceMode();
    wait(125);
    //    errorstatus=1;
    errorstatus |=(1 << 0);
    psm_status=0;
    errorstatus |=(1 << 4); 
  }
  if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
    errorstatus |=(1 << 0);
    errorstatus |=(1 << 1);
    Serial.flush();
    resetGPS();
    wait(125);
    setupGPS();
  }
  thor_wait();  
  prepare_data();
  buildstring();
  thor_string(_txstring);

}   

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  int gps_set_sucess=0;
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9                                                  };
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  while(!gps_set_sucess)
  {
    sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNMEAoff);
    if(!gps_set_sucess)
    {
      blinkled(2);
    }
  }
  wait(500);
  setGPS_DynamicModel6();
  wait(500);
  setGps_MaxPerformanceMode();
  wait(500);
}
void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}
void buildstring()
{
  if(alt>maxalt && sats >= 4)
  {
    maxalt=alt;
  }

  snprintf_P(_txstring, 80, PSTR("$$" CALLSIGN ",%li,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%i,%02x"), count++,hour, minute, second,  lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",
  lon_int,lon_dec,  maxalt,sats,battvaverage,errorstatus);


  crccat(_txstring);
  maxalt=0;
}

uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84                                                                                                         };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }
  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t ubxb;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      ubxb = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (ubxb == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                                                                                };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC                                                                                                         };
  while(!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm6);
  }
}

void setGPS_DynamicModel3()
{
  int gps_set_sucess=0;
  uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76                                                                                                         };
  while(!gps_set_sucess)
  {
    sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A                                                                                                                                                            };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    if(sats<4)
    {
      lat=0; // This just stops garbage location data from the Ublox. 
      lon=0; // If I don't know where I am don't make it up.
      alt=0; 
    }
    else
    {
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    }
    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/100; // Report to 5 decimal places
    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/100; // If you change this don't for get to amend the padding in the snprintf line.
    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67                                                                                                                                                          };
  sendUBX(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(buf[22] > 23 || buf[23] > 59 || buf[24] > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92                                                                                                                         }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91                                                                                                                         }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}
void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5                                                                                               };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void prepare_data() {

  gps_check_lock();
  gps_get_position();
  gps_get_time();

  batteryadc_v=analogRead(BATTERY_ADC);
  battv = batteryadc_v*2;
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = battv;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;
}


void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(STATUS_LED,HIGH);
    wait(100);
    digitalWrite(STATUS_LED,LOW);
    wait(100);
  }    
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}



void checkDynamicModel() {
  if(alt<=1000&&sats>4) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);      
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3);

    }
  }
}

uint16_t crccat(char *msg)
{
  uint16_t x;


  while(*msg == '$') msg++;


  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);


  snprintf_P(msg, 8, PSTR("*%04X\n"), x);

  return(x);
}

ISR(TIMER1_COMPA_vect)
{
  static uint16_t code;
  static uint8_t tone = 0;
  static uint8_t len = 0;
   uint8_t dacbuf[3];
   uint16_t dacval;
  uint8_t i, bit_sh;

  /* Transmit the tone */
//  si_set_channel(tone * THOR_DS);

  dacval = radio2dac + (tone * DOMEXSPACING);
  dacbuf[0] = 0x03;
  dacbuf[1] = dacval >> 8;
  dacbuf[2] = dacval & 0xFF;
  _tw_write(DAC, dacbuf, 3);

  if(_preamble)
  {
    tone = (tone + 2);
    if(tone >= TONES) tone -= TONES;
    _preamble--;
    return;
  }

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
    _conv_sh = (_conv_sh << 1) | ((code >> --len) & 1);
    bit_sh = (bit_sh << 2)
      | (_parity(_conv_sh & THOR_POLYA) << 1)
        | _parity(_conv_sh & THOR_POLYB);
  }

  /* Add the new data to the interleaver */
  _wtab(_inter_offset +   0, bit_sh & 0x08);
  _wtab(_inter_offset +  41, bit_sh & 0x04);
  _wtab(_inter_offset +  82, bit_sh & 0x02);
  _wtab(_inter_offset + 123, bit_sh & 0x01);

  /* Read next symbol to transmit from the interleaver */
  bit_sh = _inter_table[_inter_offset >> 3];
  if(_inter_offset & 7) bit_sh &= 0x0F;
  else bit_sh >>= 4;

  /* Shift the interleaver table offset forward */
  _inter_offset = (_inter_offset + INTER_SIZE);
  if(_inter_offset >= INTER_LEN) _inter_offset -= INTER_LEN;

  /* Calculate the next tone */
  tone = (tone + 2 + bit_sh);
  if(tone >= TONES) tone -= TONES;
}
void setMTX2Frequency()
{
  float _mtx2comp;
  int _mtx2int;
  long _mtx2fractional;
  char _mtx2command[17];
  MTX2_EN.begin(9600);
  _mtx2comp=(MTX2_FREQ+0.0015)/6.5;
  _mtx2int=_mtx2comp;
  _mtx2fractional=float(((_mtx2comp-_mtx2int)+1)*524288);
  snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
  delay(100);
  MTX2_EN.print(_mtx2command);
  delay(50);
  MTX2_EN.end();
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
void initialise_interrupt() 
{
cli();   
TCCR2A = 0;     // set entire TCCR1A register to 0
  TCCR2B = 0;     // same for TCCR1B
  OCR2A = F_CPU / 1024 / 50 - 1;  // set compare match register to desired timer count:
  //OCR2A = 155;
  TCCR2A |= (1 << WGM21);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR2B |= (1 << CS20);
  TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS22);
  // enable timer compare interrupt:
  TIMSK2 |= (1 << OCIE1A);
  
  sei();          // enable global interrupts
}

ISR(TIMER2_COMPA_vect)
{

  if(alt>1000 && sats >= 4)
  {
    digitalWrite(LED_WARN,LOW);  
    digitalWrite(LED_OK,LOW);  
  }
  else 
  {
    currentMillis = millis();
    if(currentMillis - previousMillis > ONE_SECOND) 
    {
      previousMillis = currentMillis;   
      if(errorstatus!=8)
      {
        digitalWrite(LED_WARN,!digitalRead(LED_WARN));
        digitalWrite(LED_OK,LOW);  
      }
      else 
      {
        digitalWrite(LED_OK, !digitalRead(LED_OK)); 
        digitalWrite(LED_WARN,LOW);    
      }
    }
  }
}


































