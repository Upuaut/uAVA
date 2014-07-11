
#include <util/twi.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <avr/io.h>
#include <SoftwareSerial.h>

#define MTX2_FREQ 434.480 // format 434.XXX  
char callsign[9] = "UAD";  // MAX 9 CHARACTERS!!

unsigned char PROGMEM varicode[][3] = {
	/* Primary alphabet */
	{ 1,15, 9}, { 1,15,10}, { 1,15,11}, { 1,15,12}, { 1,15,13}, { 1,15,14}, { 1,15,15}, { 2, 8, 8},
	{ 2,12, 0}, { 2, 8, 9}, { 2, 8,10}, { 2, 8,11}, { 2, 8,12}, { 2,13, 0}, { 2, 8,13}, { 2, 8,14},
	{ 2, 8,15}, { 2, 9, 8}, { 2, 9, 9}, { 2, 9,10}, { 2, 9,11}, { 2, 9,12}, { 2, 9,13}, { 2, 9,14},
	{ 2, 9,15}, { 2,10, 8}, { 2,10, 9}, { 2,10,10}, { 2,10,11}, { 2,10,12}, { 2,10,13}, { 2,10,14},
	{ 0, 0, 0}, { 7,11, 0}, { 0, 8,14}, { 0,10,11}, { 0, 9,10}, { 0, 9, 9}, { 0, 8,15}, { 7,10, 0},
	{ 0, 8,12}, { 0, 8,11}, { 0, 9,13}, { 0, 8, 8}, { 2,11, 0}, { 7,14, 0}, { 7,13, 0}, { 0, 8, 9},
	{ 3,15, 0}, { 4,10, 0}, { 4,15, 0}, { 5, 9, 0}, { 6, 8, 0}, { 5,12, 0}, { 5,14, 0}, { 6,12, 0},
	{ 6,11, 0}, { 6,14, 0}, { 0, 8,10}, { 0, 8,13}, { 0,10, 8}, { 7,15, 0}, { 0, 9,15}, { 7,12, 0},
	{ 0, 9, 8}, { 3, 9, 0}, { 4,14, 0}, { 3,12, 0}, { 3,14, 0}, { 3, 8, 0}, { 4,12, 0}, { 5, 8, 0},
	{ 5,10, 0}, { 3,10, 0}, { 7, 8, 0}, { 6,10, 0}, { 4,11, 0}, { 4, 8, 0}, { 4,13, 0}, { 3,11, 0},
	{ 4, 9, 0}, { 6,15, 0}, { 3,13, 0}, { 2,15, 0}, { 2,14, 0}, { 5,11, 0}, { 6,13, 0}, { 5,13, 0},
	{ 5,15, 0}, { 6, 9, 0}, { 7, 9, 0}, { 0,10,14}, { 0,10, 9}, { 0,10,15}, { 0,10,10}, { 0, 9,12},
	{ 0, 9,11}, { 4, 0, 0}, { 1,11, 0}, { 0,12, 0}, { 0,11, 0}, { 1, 0, 0}, { 0,15, 0}, { 1, 9, 0},
	{ 0,10, 0}, { 5, 0, 0}, { 2,10, 0}, { 1,14, 0}, { 0, 9, 0}, { 0,14, 0}, { 6, 0, 0}, { 3, 0, 0}, 
	{ 1, 8, 0}, { 2, 8, 0}, { 7, 0, 0}, { 0, 8, 0}, { 2, 0, 0}, { 0,13, 0}, { 1,13, 0}, { 1,12, 0}, 
	{ 1,15, 0}, { 1,10, 0}, { 2, 9, 0}, { 0,10,12}, { 0, 9,14}, { 0,10,12}, { 0,11, 8}, { 2,10,15}, 
	{ 2,11, 8}, { 2,11, 9}, { 2,11,10}, { 2,11,11}, { 2,11,12}, { 2,11,13}, { 2,11,14}, { 2,11,15}, 
	{ 2,12, 8}, { 2,12, 9}, { 2,12,10}, { 2,12,11}, { 2,12,12}, { 2,12,13}, { 2,12,14}, { 2,12,15}, 
	{ 2,13, 8}, { 2,13, 9}, { 2,13,10}, { 2,13,11}, { 2,13,12}, { 2,13,13}, { 2,13,14}, { 2,13,15}, 
	{ 2,14, 8}, { 2,14, 9}, { 2,14,10}, { 2,14,11}, { 2,14,12}, { 2,14,13}, { 2,14,14}, { 2,14,15}, 
	{ 0,11, 9}, { 0,11,10}, { 0,11,11}, { 0,11,12}, { 0,11,13}, { 0,11,14}, { 0,11,15}, { 0,12, 8}, 
	{ 0,12, 9}, { 0,12,10}, { 0,12,11}, { 0,12,12}, { 0,12,13}, { 0,12,14}, { 0,12,15}, { 0,13, 8}, 
	{ 0,13, 9}, { 0,13,10}, { 0,13,11}, { 0,13,12}, { 0,13,13}, { 0,13,14}, { 0,13,15}, { 0,14, 8}, 
	{ 0,14, 9}, { 0,14,10}, { 0,14,11}, { 0,14,12}, { 0,14,13}, { 0,14,14}, { 0,14,15}, { 0,15, 8}, 
	{ 0,15, 9}, { 0,15,10}, { 0,15,11}, { 0,15,12}, { 0,15,13}, { 0,15,14}, { 0,15,15}, { 1, 8, 8}, 
	{ 1, 8, 9}, { 1, 8,10}, { 1, 8,11}, { 1, 8,12}, { 1, 8,13}, { 1, 8,14}, { 1, 8,15}, { 1, 9, 8}, 
	{ 1, 9, 9}, { 1, 9,10}, { 1, 9,11}, { 1, 9,12}, { 1, 9,13}, { 1, 9,14}, { 1, 9,15}, { 1,10, 8}, 
	{ 1,10, 9}, { 1,10,10}, { 1,10,11}, { 1,10,12}, { 1,10,13}, { 1,10,14}, { 1,10,15}, { 1,11, 8}, 
	{ 1,11, 9}, { 1,11,10}, { 1,11,11}, { 1,11,12}, { 1,11,13}, { 1,11,14}, { 1,11,15}, { 1,12, 8}, 
	{ 1,12, 9}, { 1,12,10}, { 1,12,11}, { 1,12,12}, { 1,12,13}, { 1,12,14}, { 1,12,15}, { 1,13, 8}, 
	{ 1,13, 9}, { 1,13,10}, { 1,13,11}, { 1,13,12}, { 1,13,13}, { 1,13,14}, { 1,13,15}, { 1,14, 8}, 
	{ 1,14, 9}, { 1,14,10}, { 1,14,11}, { 1,14,12}, { 1,14,13}, { 1,14,14}, { 1,14,15}, { 1,15, 8},

	/* Secondary alphabet */
	{ 6,15, 9}, { 6,15,10}, { 6,15,11}, { 6,15,12}, { 6,15,13}, { 6,15,14}, { 6,15,15}, { 7, 8, 8},
	{ 4,10,12}, { 7, 8, 9}, { 7, 8,10}, { 7, 8,11}, { 7, 8,12}, { 4,10,13}, { 7, 8,13}, { 7, 8,14},
	{ 7, 8,15}, { 7, 9, 8}, { 7, 9, 9}, { 7, 9,10}, { 7, 9,11}, { 7, 9,12}, { 7, 9,13}, { 7, 9,14},
	{ 7, 9,15}, { 7,10, 8}, { 7,10, 9}, { 7,10,10}, { 7,10,11}, { 7,10,12}, { 7,10,13}, { 7,10,14},
	{ 3, 8, 8}, { 4,15,11}, { 5, 8,14}, { 5,10,11}, { 5, 9,10}, { 5, 9, 9}, { 5, 8,15}, { 4,15,10},
	{ 5, 8,12}, { 5, 8,11}, { 5, 9,13}, { 5, 8, 8}, { 4,10,11}, { 4,15,14}, { 4,15,13}, { 5, 8, 9},
	{ 4,11,15}, { 4,12,10}, { 4,12,15}, { 4,13, 9}, { 4,14, 8}, { 4,13,12}, { 4,13,14}, { 4,14,12},
	{ 4,14,11}, { 4,14,14}, { 5, 8,10}, { 5, 8,13}, { 5,10, 8}, { 4,15,15}, { 5, 9,15}, { 4,15,12},
	{ 5, 9, 8}, { 4,11, 9}, { 4,12,14}, { 4,11,12}, { 4,11,14}, { 4,11, 8}, { 4,12,12}, { 4,13, 8},
	{ 4,13,10}, { 4,11,10}, { 4,15, 8}, { 4,14,10}, { 4,12,11}, { 4,12, 8}, { 4,12,13}, { 4,11,11},
	{ 4,12, 9}, { 4,14,15}, { 4,11,13}, { 4,10,15}, { 4,10,14}, { 4,13,11}, { 4,14,13}, { 4,13,13},
	{ 4,13,15}, { 4,14, 9}, { 4,15, 9}, { 5,10,14}, { 5,10, 9}, { 5,10,15}, { 5,10,10}, { 5, 9,12},
	{ 5, 9,11}, { 3, 8,12}, { 4, 9,11}, { 4, 8,12}, { 4, 8,11}, { 3, 8, 9}, { 4, 8,15}, { 4, 9, 9},
	{ 4, 8,10}, { 3, 8,13}, { 4,10,10}, { 4, 9,14}, { 4, 8, 9}, { 4, 8,14}, { 3, 8,14}, { 3, 8,11},
	{ 4, 9, 8}, { 4,10, 8}, { 3, 8,15}, { 4, 8, 8}, { 3, 8,10}, { 4, 8,13}, { 4, 9,13}, { 4, 9,12},
	{ 4, 9,15}, { 4, 9,10}, { 4,10, 9}, { 5,10,12}, { 5, 9,14}, { 5,10,12}, { 5,11, 8}, { 7,10,15},
	{ 7,11, 8}, { 7,11, 9}, { 7,11,10}, { 7,11,11}, { 7,11,12}, { 7,11,13}, { 7,11,14}, { 7,11,15},
	{ 7,12, 8}, { 7,12, 9}, { 7,12,10}, { 7,12,11}, { 7,12,12}, { 7,12,13}, { 7,12,14}, { 7,12,15},
	{ 7,13, 8}, { 7,13, 9}, { 7,13,10}, { 7,13,11}, { 7,13,12}, { 7,13,13}, { 7,13,14}, { 7,13,15},
	{ 7,14, 8}, { 7,14, 9}, { 7,14,10}, { 7,14,11}, { 7,14,12}, { 7,14,13}, { 7,14,14}, { 7,14,15},
	{ 5,11, 9}, { 5,11,10}, { 5,11,11}, { 5,11,12}, { 5,11,13}, { 5,11,14}, { 5,11,15}, { 5,12, 8},
	{ 5,12, 9}, { 5,12,10}, { 5,12,11}, { 5,12,12}, { 5,12,13}, { 5,12,14}, { 5,12,15}, { 5,13, 8},
	{ 5,13, 9}, { 5,13,10}, { 5,13,11}, { 5,13,12}, { 5,13,13}, { 5,13,14}, { 5,13,15}, { 5,14, 8},
	{ 5,14, 9}, { 5,14,10}, { 5,14,11}, { 5,14,12}, { 5,14,13}, { 5,14,14}, { 5,14,15}, { 5,15, 8},
	{ 5,15, 9}, { 5,15,10}, { 5,15,11}, { 5,15,12}, { 5,15,13}, { 5,15,14}, { 5,15,15}, { 6, 8, 8},
	{ 6, 8, 9}, { 6, 8,10}, { 6, 8,11}, { 6, 8,12}, { 6, 8,13}, { 6, 8,14}, { 6, 8,15}, { 6, 9, 8},
	{ 6, 9, 9}, { 6, 9,10}, { 6, 9,11}, { 6, 9,12}, { 6, 9,13}, { 6, 9,14}, { 6, 9,15}, { 6,10, 8},
	{ 6,10, 9}, { 6,10,10}, { 6,10,11}, { 6,10,12}, { 6,10,13}, { 6,10,14}, { 6,10,15}, { 6,11, 8},
	{ 6,11, 9}, { 6,11,10}, { 6,11,11}, { 6,11,12}, { 6,11,13}, { 6,11,14}, { 6,11,15}, { 6,12, 8},
	{ 6,12, 9}, { 6,12,10}, { 6,12,11}, { 6,12,12}, { 6,12,13}, { 6,12,14}, { 6,12,15}, { 6,13, 8},
	{ 6,13, 9}, { 6,13,10}, { 6,13,11}, { 6,13,12}, { 6,13,13}, { 6,13,14}, { 6,13,15}, { 6,14, 8},
	{ 6,14, 9}, { 6,14,10}, { 6,14,11}, { 6,14,12}, { 6,14,13}, { 6,14,14}, { 6,14,15}, { 6,15, 8},
};
#define DAC (0x10) // DAC I2C Address is 16
#define STATUSLED 6
#define RADIO_ENABLE 2
//#define BAUDRATE 15.625	// DominoEX16
//#define DOMEXSPACING 0x4B // EX4  
//#define DOMEXSPACING 0x96 // EX8  OK
//#define DOMEXSPACING 0x96 // EX16 OK 
#define DOMEXSPACING 0xCF // EX22 OK
#define LED_WARN 5
#define LED_OK 6
#define BATTERY_ADC A0
#define POWERSAVING
#define ONE_SECOND F_CPU / 1024 / 8
SoftwareSerial MTX2_EN(12, RADIO_ENABLE); // RX, TX

static char _txstring[120];
static volatile char _txstatus = 0;

unsigned int radio2dac = 0x4FC0;
unsigned int dom = 0x0000;

int errorstatus=0;
/* Error Status Bit Level Field :
 Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = Not used
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 
 So error 8 means the everything is fine just the GPS is in pedestrian mode. 
 Below 1000 meters the code puts the GPS in the more accurate pedestrian mode. 
 Above 1000 meters it switches to dynamic model 6 i.e flight mode and turns the LED's off for additional power saving. 
 So as an example error code 40 = 101000 means GPS not locked and in pedestrian mode. 
 */
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int GPSerror = 0,navmode = 0,psm_status = 0,lat_int=0,lon_int=0;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;
unsigned long currentMillis;
long previousMillis = 0;
float batteryadc_v;
float battvsmooth[5];
int battvaverage=0;
unsigned long startTime;
volatile int count=1;
uint8_t buf[60]; 

void setup()
{
  TWSR = 0;
  TWBR = ((F_CPU / 100000L) - 16) / 2;
  TWSR = _BV(TWPS1) | _BV(TWPS0); /* /64 prescaler */
  TWBR = 0;
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
  Serial.begin(9600);
  pinMode(STATUSLED, OUTPUT);     
  pinMode(RADIO_ENABLE, OUTPUT);     
  digitalWrite(RADIO_ENABLE,HIGH);
  pinMode(LED_WARN, OUTPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(BATTERY_ADC, INPUT);
  setMTX2Frequency();
  resetGPS();
  setupGPS();
  initialise_interrupt();
}

void loop()
{
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();

  if(lock!=3)
  {
    errorstatus |=(1 << 5);  // Set bit 5 (Lock 0 = GPS Locked 1= Not Locked)
  }
  else
  {
    errorstatus &= ~(1 << 5); // Unset bit 5 (Lock 0 = GPS Locked 1= Not Locked)
  }
  checkDynamicModel();

#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))
  {
    setGPS_PowerSaveMode();
    delay(1000);
    //    pinMode(LED_WARN, INPUT); 
    psm_status=1;
    errorstatus &= ~(1 << 4);
  }
#endif

  prepare_data();

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
    delay(125);
    setGps_MaxPerformanceMode();
    delay(125);
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
    delay(125);
    setupGPS();
  }


  if(_txstatus==0)
  {
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    } 
    snprintf(_txstring,100, "$$$$$%s,%i,%02d:%02d:%02d,%s%i.%06ld,%s%i.%06ld,%ld,%d,%i,%02x",callsign,count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,battvaverage,errorstatus);
    crccat(_txstring);
    maxalt=0;
    _txstatus = 1;
    count++;
  }    
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

void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  //  OCR1A = F_CPU / 1024 / BAUDRATE - 1;  // set compare match register to desired timer count:
  //	OCR1A = F_CPU/16000-1; // DOMINOEX16
  OCR1A = F_CPU/22050-1; // DOMINOEX22
  //	OCR1A = F_CPU/8000-1; // DOMINOEX8
  //	OCR1A = F_CPU/4000-1; // DOMINOEX4
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  
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

ISR(TIMER1_COMPA_vect)
{
  //  digitalWrite(STATUSLED, !digitalRead(STATUSLED)); 
  static uint8_t sym = 0;  /* Currently transmitting symbol */
  static uint8_t c = 0x00; /* Current character */
  static uint8_t s = 0;    /* Current symbol index */
  static char *p = 0;      /* Pointer to primary tx buffer */

  uint8_t nsym;
  uint8_t dacbuf[3];
  uint16_t dacval;

  /* Fetch the next symbol */
  nsym = pgm_read_byte(&varicode[c][s++]);

  /* Update the transmitting symbol */
  sym = (sym + 2 + nsym) % 18;

  /* Send it to the DAC */
  dacval = radio2dac + (sym * DOMEXSPACING);
  dacbuf[0] = 0x03;
  dacbuf[1] = dacval >> 8;
  dacbuf[2] = dacval & 0xFF;
  _tw_write(DAC, dacbuf, 3);

  /* Check if this character has less than 3 symbols */
  if(s < 3 && !(pgm_read_byte(&varicode[c][s]) & 0x08)) s = 3;

  /* Still more symbols to send? Exit interrupt */
  if(s != 3) return;

  /* We're done with this character, fetch the next one */
  s = 0;

  switch(_txstatus)
  {
  case 0:
    /* Nothing is ready for us, transmit a NUL */
    c = 0x00;
    break;

  case 1:
    /* 1 signals a new primary string is ready */
    p = _txstring;
    _txstatus++;
    /* Fall through... */

  case 2:
    /* Read the next character from the primary buffer */
    c = (uint8_t) *(p++);

    /* Reached the end of the string? */
    if(c == 0x00) _txstatus = 0;

    break;
  }
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
void resetGPS() {
  uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5           };
  sendUBX(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}
void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  delay(100);
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}
void setupGPS() {
  // Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  int gps_set_sucess=0;
  uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9           };
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
  delay(500);
  setGPS_DynamicModel6();
  delay(500);
  setGps_MaxPerformanceMode();
  delay(500);
}
void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC           };
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
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76           };
  while(!gps_set_sucess)
  {
    sendUBX(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setdm3);
  }
}
void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91           }; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax)/sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
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
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(LED_WARN,HIGH);
    delay(100);
    digitalWrite(LED_WARN,LOW);
    delay(100);
  }    
}  

uint8_t gps_check_nav(void)
{
  uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84           };
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
void checkDynamicModel() {
  if(alt<=1000&&sats>4) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);  // Set Bit 3 indicating we are in pedestrian mode    
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3); // Unset bit 3 indicating we are in flight mode

    }
  }
}
void setGPS_PowerSaveMode() {
  // Power Save Mode 
  uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92           }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}
void prepare_data() {
  gps_check_lock();
  gps_get_position();
  gps_get_time();
  batteryadc_v=float(analogRead(BATTERY_ADC)*3.5);
  battvsmooth[4] = battvsmooth[3];
  battvsmooth[3] = battvsmooth[2];
  battvsmooth[2] = battvsmooth[1];
  battvsmooth[1] = battvsmooth[0];
  battvsmooth[0] = batteryadc_v;
  battvaverage = (battvsmooth[0]+battvsmooth[1]+ battvsmooth[2]+battvsmooth[3]+battvsmooth[4])/5;
  //battvaverage=batteryadc_v;


}

void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16                                                                                                                                                                  };
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
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A           };
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
      lat=0;
      lon=0;
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
    // 4 bytes of latitude/longitude (1e-7)
    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/10;
    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/10;


    // 4 bytes of altitude above MSL (mm)

    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67           };
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

uint16_t crccat(char *msg)
{
  uint16_t x;


  while(*msg == '$') msg++;


  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);


  snprintf_P(msg, 8, PSTR("*%04X\n"), x);

  return(x);
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
