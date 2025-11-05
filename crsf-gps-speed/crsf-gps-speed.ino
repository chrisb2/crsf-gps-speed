#include "CRSFforArduino.hpp"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "wiring_private.h"
#include <ezLED.h>

#define GPS_BAUD_RATE 19200
#define GPS_TX D10
#define GPS_RX D9
#define GPS_ENABLE D8
#define GPS_ENABLE_CHAN 4 // AUX1
#define RED_LED D0
#define BLUE_LED D1
#define GREEN_LED D2
#define GPS_DEBUG_ENABLED 1
//#define SERIAL_DEBUG_ENABLED 1

int incomingByte = 0;
bool gpsEnabled = false;
CRSFforArduino crsf = CRSFforArduino(&Serial1);
TinyGPSPlus gps;
Uart gpsSerial(&sercom0, GPS_RX, GPS_TX, SERCOM_RX_PAD_1, UART_TX_PAD_2);
ezLED redLed(RED_LED);
ezLED blueLed(BLUE_LED);
ezLED greenLed(GREEN_LED);

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A", "E", "T", "R",
    "Aux1", "Aux2", "Aux3", "Aux4",
    "Aux5", "Aux6", "Aux7", "Aux8",
    "Aux9", "Aux10", "Aux11", "Aux12"};
    
void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);
void enableGPS();
void disableGPS();
void displayInfo();
void sendDataToReceiver();

void setup() {
  Serial.begin(460800);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPS_ENABLE, OUTPUT);

  pinPeripheral(GPS_TX, PIO_SERCOM_ALT);
  pinPeripheral(GPS_RX, PIO_SERCOM_ALT);

  if (crsf.begin()) {
    redLed.blink(5, 2000);
  } else {
    redLed.turnON();
  }

  rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;
  crsf.setRcChannelsCallback(onReceiveRcChannels);

  enableGPS();  // TODO disable?
}

void loop() {
  redLed.loop();
  blueLed.loop();
  greenLed.loop();
  crsf.update();

  while (gpsSerial.available() > 0) {
    incomingByte = gpsSerial.read();
    if (gps.encode(incomingByte)) {
      sendDataToReceiver();
    }

#ifdef SERIAL_DEBUG_ENABLED
    if (incomingByte == '$') {
      Serial.println();
    }
    Serial.print(incomingByte, HEX);
    Serial.print(' ');
#endif
  }
}

void sendDataToReceiver() {
  blueLed.blinkNumberOfTimes(2, 2, 1);
  if (gps.location.isUpdated()) {
    crsf.telemetryWriteGPS(gps.location.lat(), gps.location.lng(), gps.altitude.value(), 
        gps.speed.mps() * 100, gps.course.deg(), gps.satellites.value());
    greenLed.blinkNumberOfTimes(5, 5, 1);
    displayInfo();
  } 
}

void enableGPS() {
  digitalWrite(GPS_ENABLE, HIGH);
  gpsSerial.begin(GPS_BAUD_RATE);
  gpsEnabled = true;
#ifdef GPS_DEBUG_ENABLED
  Serial.println(F("GPS enabled"));
#endif
}

void disableGPS() {
  //gpsSerial.end(); TODO
  digitalWrite(GPS_ENABLE, LOW);
  gpsEnabled = false;
#ifdef GPS_DEBUG_ENABLED
  Serial.println(F("GPS disabled"));
#endif
}

void printChannelValue(uint16_t val) {
#ifdef GPS_DEBUG_ENABLED
  Serial.print(F("GPS enable channel value: "));
  Serial.println(val);
#endif
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcData) {
  if (rcData->failsafe) {
#ifdef GPS_DEBUG_ENABLED
    Serial.println(F("Failsafe!"));
#endif
    //disableGPS(); TODO
  }

  uint16_t gpsChanValue = crsf.rcToUs(rcData->value[GPS_ENABLE_CHAN]);

  if (gpsChanValue == 1000 && gpsEnabled) {
    printChannelValue(gpsChanValue);
    disableGPS();
  } else if (gpsChanValue == 2000 && !gpsEnabled) {
    printChannelValue(gpsChanValue);
    enableGPS();
  }
}

void displayInfo() {
#ifdef GPS_DEBUG_ENABLED
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(","));
  Serial.print(gps.location.lng(), 6);
  Serial.print(F(","));
  Serial.print(gps.speed.kmph(), 1);
  Serial.print(F(","));
  Serial.print(gps.altitude.meters(), 0);
  Serial.print(F(","));
  Serial.print(gps.course.deg(), 0);
  Serial.print(F(","));
  Serial.print(gps.satellites.value());
  Serial.print(F(","));
  Serial.println(gps.hdop.value());
#endif
}

void SERCOM0_Handler()
{
  gpsSerial.IrqHandler();
}
