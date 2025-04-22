#include "CRSFforArduino.hpp"
#include <TinyGPSPlus.h>
#include <ezLED.h>
#include <SoftwareSerial.h>

#define GPS_BAUD_RATE 19200
#define GPS_RX 3
#define GPS_TX 4
#define GREEN_LED 16
#define GPS_DEBUG_ENABLED 1
// #define SERIAL_DEBUG_ENABLED 1

int incomingByte = 0;
CRSFforArduino crsf = CRSFforArduino(&Serial1);
TinyGPSPlus gps;
SoftwareSerial gpsSerial = SoftwareSerial(GPS_RX, GPS_TX);
ezLED crsfLed(LED_BUILTIN);
ezLED gpsFixLed(GREEN_LED);

void setup() {
  Serial.begin(460800);
  gpsSerial.begin(GPS_BAUD_RATE);

  if (crsf.begin()) {
    crsfLed.blink(250, 750);
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

void sendDataToReceiver() {
  if (gps.location.isValid()) {
    if (gps.location.isUpdated()) {
      crsf.telemetryWriteGPS(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), 
        gps.speed.kmph(), gps.course.deg(), gps.satellites.value());
      crsf.update();
      displayInfo();
      gpsFixLed.toggle();
    }
  }
}

void loop() {
  crsfLed.loop();
  gpsFixLed.loop();

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
