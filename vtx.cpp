#include "vtx.h"
#include <Arduino.h>
#include "HCT138.h"
#include <SPI.h>

// global variables
// Channels to sent to the SPI registers
const uint16_t channelTable[] PROGMEM = {
  // Channel 1 - 8
  0x2A05,    0x299B,    0x2991,    0x2987,    0x291D,    0x2913,    0x2909,    0x289F,    // Band A
  0x2903,    0x290C,    0x2916,    0x291F,    0x2989,    0x2992,    0x299C,    0x2A05,    // Band B
  0x2895,    0x288B,    0x2881,    0x2817,    0x2A0F,    0x2A19,    0x2A83,    0x2A8D,    // Band E
  0x2906,    0x2910,    0x291A,    0x2984,    0x298E,    0x2998,    0x2A02,    0x2A0C,    // Band F / Airwave
  0x281d,    0x2890,    0x2902,    0x2915,    0x2987,    0x299a,    0x2a0c,    0x2a1f     // IRC Race Band
};

// Channels with their Mhz Values
const uint16_t channelFreqTable[] PROGMEM = {
  // Channel 1 - 8
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,  // Band F / Airwave
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Race Band
};

const uint8_t bandNames[] PROGMEM = { // faster than calculate
  'A', 'A', 'A', 'A', 'A', 'A', 'A', 'A',
  'B', 'B', 'B', 'B', 'B', 'B', 'B', 'B',
  'E', 'E', 'E', 'E', 'E', 'E', 'E', 'E',
  'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F',
  'R', 'R', 'R', 'R', 'R', 'R', 'R', 'R'
};
const uint8_t bandNumber[] PROGMEM = { // faster than calculate
  0, 0, 0, 0, 0, 0, 0, 0,
  1, 1, 1, 1, 1, 1, 1, 1,
  2, 2, 2, 2, 2, 2, 2, 2,
  3, 3, 3, 3, 3, 3, 3, 3,
  4, 4, 4, 4, 4, 4, 4, 4,
};
// Symbol for each channel
const uint8_t channelSymbol[] PROGMEM = {
  0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, // Band A
  0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, // Band B
  0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, // Band E
  0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, // Band F
  0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7 // Band RACE
};

uint8_t channelList[CHANNEL_MAX_INDEX + 1] = {};

int spiClockPin = D5;
int spiDataPin = D7;
int slaveSelectPin = D8;

void setupChannelList(){
  // create channelList lookup sorted by freqency
  // 1. fill array
  for (int i = 0; i <= CHANNEL_MAX_INDEX; i++)
  {
    channelList[i] = i;
  }
  // Bubble sort algorythm by value of freqency reference
  for (int i = 0; i < CHANNEL_MAX_INDEX; i++) {
    for (int o = 0; o < ((CHANNEL_MAX_INDEX + 1) - (i + 1)); o++)
    {
      if (pgm_read_word_near(channelFreqTable + channelList[o]) > pgm_read_word_near(channelFreqTable + channelList[o + 1])) // compare two elements to each other and swap on demand
      {
        // swap index;
        int t = channelList[o];
        channelList[o] = channelList[o + 1];
        channelList[o + 1] = t;
      }
    }
  }
}

/*
void spi_32_transfer(uint32_t value){
  uint8_t* buffer = (uint8_t*) &value; // for simple byte access
  
  //Spi.mode((1<<DORD) | (1<<SPR0));  // set to SPI LSB first mode  and to 1/64 speed
  SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
  
  //digitalWrite(rx5808_SEL, LOW); // select
  setPinLow(1);
  delayMicroseconds(1); 
  
  SPI.transfer(*(buffer + 0)); // byte 0

  SPI.transfer(*(buffer + 1)); // byte 1

  SPI.transfer(*(buffer + 2)); // byte 2

  SPI.transfer(*(buffer + 3)); // byte 3
  
  //digitalWrite(rx5808_SEL,HIGH); // transfer done
  delayMicroseconds(1);
  setPinHigh(1);
  delayMicroseconds(1);     
  SPI.endTransaction();
  
  //Spi.mode(0);  // set SPI mode back to MSB first (used by OSD)
}

void setChannelModule__(uint8_t channel){
  uint16_t channelData;
  channelData = pgm_read_word_near(channelTable + channel);
  
  uint32_t buffer32=0;
  uint8_t address = 0;
  uint32_t data = 0;
  uint8_t write = 0;
  
  // note to SPI of rx5808 chip (RTC6715)
  // The SPI interface needs 25 bits to be written.
  // The HW SPI controller of the Atmel has 8 bit.
  // Since 8 does not fit 8 bit, we must transfer "more" bits (32 bit)
  // This is possible because the shift register of the RTC6715 will 
  // use the "last" bits when CS goes passive.
  // This means, that we transfer 7x dummy '0' + 25 bits = 32 bits.
  
  // data format 25 bits
  // Order: A0-3, !R/W, D0-D19
  // NOTE: LSB first!
  
  // This order is required:
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  
  // LSB first
  // 1. send 7bit x dummy '0'
  // 2. 4 bit adress
  // 3. RW bit
  // 4. 20 bit data
  
  // for efficient code the follwing data structre in a uint32_t is used:
  
  // MSB                           LSB  
  // D19...D0 + RW + A3...A0 + 0000000
  //    20 bit + 1 bit + 4 bit + 7 bit = 32 bit
  // DDDDDDDDDDDDDDDDDDDDRAAAA0000000
  //                    ^
  //                    12
  //                     ^
  //                     11  ^
  //                         7 
  // SPI must set to DORD = 1 (Data Order LSB first
  
  // read operation ??? CHECK
 
  address = 0x1;
  data = channelData;
  write = 1;

  buffer32=0; //  init buffer to 0
  buffer32=((data & 0xfffff) << 12 ) | ((write & 1)<<11) | ((address & 0xf) <<7);
  spi_32_transfer(buffer32);
}
*/


void setChannelModule(uint8_t channel)
{
  uint8_t i;
  uint16_t channelData;
  
  //channelData = pgm_read_word(&channelTable[channel]);
  //channelData = channelTable[channel];
  channelData = pgm_read_word_near(channelTable + channel);

//   osd_print_debug_x (1, 1, "TUNE:", channelData);  
//  SPCR = 0; // release SPI controller for bit banging

  
  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }

    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--)
    SERIAL_SENDBIT0();

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();

  
 // Spi.mode(0);  // set SPI mode back to MSB first (used by OSD)  
  
}

void SERIAL_SENDBIT1()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0()
{
  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);

  digitalWrite(spiDataPin, LOW);
  delayMicroseconds(1);
  digitalWrite(spiClockPin, HIGH);
  delayMicroseconds(1);

  digitalWrite(spiClockPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, LOW);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH()
{
  delayMicroseconds(1);
  digitalWrite(slaveSelectPin, HIGH);
  delayMicroseconds(1);
}
/*
//  does fast scan in backgroud and sets spectrum data
// runtime ~1 second on 40 channels
void background_scan(uint8_t size)
{
    uint8_t channel_scan=0;
    uint8_t channel_index;
    uint8_t current_rssi_max=0; // set seek values
    for (channel_scan=CHANNEL_MIN_INDEX; channel_scan <= CHANNEL_MAX_INDEX;channel_scan++ )
    {
       // osd_print_debug(1,1,"CH:",channel_scan);
        channel_index = channelList[channel_scan];            
        setChannelModule(channel_index);   // TUNE 
        time_of_tune=millis();                
        wait_rssi_ready();
        rssi = readRSSI();
        if(rssi>current_rssi_max){
            current_rssi_max=rssi;
        }
        // add spectrum of current channel
       //spectrum_add_column (size, pgm_read_word_near(channelFreqTable + channel_index), rssi,0);                
    }
    // set new seek threshold
    rssi_seek_threshold=(uint16_t)current_rssi_max*RSSI_SEEK_TRESHOLD/100;
}
  

uint8_t channel_from_index(uint8_t channelIndex)
{
    uint8_t loop=0;
    uint8_t channel=0;
    for (loop=0;loop<=CHANNEL_MAX;loop++)
    {                
    //if(pgm_read_byte_near(channelList + loop) == channelIndex)
    if(channelList[loop] == channelIndex)
        {
            channel=loop;
            break;
        }
    }
    return (channel);
}   

void wait_rssi_ready()
{
    // CHECK FOR MINIMUM DELAY
    // check if RSSI is stable after tune by checking the time
    uint16_t tune_time = millis()-time_of_tune;
    // module need >20ms to tune.
    // 30 ms will to a 32 channel scan in 1 second.
    if(tune_time < MIN_TUNE_TIME)
    {
        // wait until tune time is full filled
        delay(MIN_TUNE_TIME-tune_time);
    }
}

// read values
// to average
// to normisation
// do statistic
uint16_t readRSSI() 
{
    uint16_t rssi = 0;
    for (uint8_t i = 0; i < 10; i++) 
    {
        rssi += analogRead(rssiPin);
    }
    rssi=rssi/10; // average
    // special case for RSSI setup
    /*if(state==STATE_RSSI_SETUP)
    { // RSSI setup
        if(rssi < rssi_setup_min)
        {
            rssi_setup_min=rssi;
        }
        if(rssi > rssi_setup_max)
        {
            rssi_setup_max=rssi;
        }    
    }   */

    /*
#if 0    
    osd_print_debug (1, 3, " min: ",rssi_min );  
    osd_print_debug (16, 3, " max: ",rssi_max );      
    osd_print_debug (1, 2, " RSSI r: ",rssi );  
#endif    
    rssi = constrain(rssi, rssi_min, rssi_max);    //original 90---250
    rssi=rssi-rssi_min; // set zero point (value 0...160)
    rssi = map(rssi, 0, rssi_max-rssi_min , 1, 100);   // scale from 1..100%
// TEST CODE    
    //rssi=random(0, 100);     
#if 0 
    osd_print_debug (16, 2, " RSSI: ",rssi );     
#endif    
    return (rssi);
}

*/


