#include <Arduino.h>

#define CHANNEL_BAND_SIZE 8
#define CHANNEL_MIN_INDEX 0
#define CHANNEL_MAX_INDEX 39

#define CHANNEL_MAX 39
#define CHANNEL_MIN 0


#define RSSI_SEEK_TRESHOLD 80

#define MIN_TUNE_TIME 30 // tune time for rssi


void setupChannelList();

//void setChannelModule__(uint8_t channel);
void setChannelModule(uint8_t channel);
void SERIAL_ENABLE_HIGH();
void SERIAL_ENABLE_LOW();
void SERIAL_SENDBIT1();
void SERIAL_SENDBIT0();


