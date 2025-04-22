# CRSF GPS Speed and Coordinates Sensor

This sensor based on a [SeeedStudio XIAO RP2040](https://www.seeedstudio.com/XIAO-RP2040-v1-0-p-5026.html) reads from a GPS module and sends GPS telemetry values using the CRSF protocol, which can be consumed by an appropriate [ELRS](https://www.expresslrs.org/) radio control receiver, for example a RadioMaster ER6 or ER8.

The XIAO RP2040 was chosen as the 3.3V buck converter (part no. RS3236) it has accepts upto 8V on *Vin*, this is required as the ER6 and ER8 have 7V on the positive pin of the CRSF input port, allowing the RP2040 to be directly powered by the receiver.

Current meassurements show this sensor takes 80-90mA with the *TOPGNSS GG-1802* GPS module used.

Note that this project uses the arduino-pico platform to enable the use of a additional Serial port on the RP2040 using PIO, see [“SoftwareSerial” PIO-based UART](https://arduino-pico.readthedocs.io/en/latest/piouart.html#).

## GPS Module Configuration

The following configuration is optimised for a yacht, you may need to alter the  *Dynamic Model* for a plane. 

The *TinyGPSPlus* library parses only the *$GPGGA* and *$GPRMC* NMEA sentences, so all others are turn off to reduce the serial load on the RP2040 at the 10Hz GPS update rate configured.

Configure the following in the *View->Configuration View* of the [u-blox u-center](https://www.u-blox.com/en/product/u-center) application:
* PRT->Baudrate: 19200
* RATE->Measurement Period: 100ms
* PMS->Setup ID: 0 - Full Power
* NAV5->Dynamic Model: 3 - Pedestrian
* GNSS->BeiDou: checked (in addition to GPS)
* MSG->Message->F0-01 NMEA GxGLL: uncheck all
* MSG->Message->F0-01 NMEA GxGSA: uncheck all
* MSG->Message->F0-01 NMEA GxGSV: uncheck all
* MSG->Message->F0-01 NMEA GxVTG: uncheck all

## RP2040 LEDs

* Blue - flashes once per second if CRSF is initialized.
* Green - flashes once per telemetry message over CRSF.

## ER6 Receiver CRSF Port

I asked RadioMaster how much current can be drawn from the ER6 CRSF input port, this was their response:

The positive and negative (+/-) pins of the CRSF port of the ER6 receiver have limited power supply capacity and are designed for low-power sensors or communication devices.

1. Power supply limitations and purpose: The CRSF port is mainly used to provide the necessary low-power power supply for sensors and is not used to drive high-current devices (such as servos). If external sensors are connected at the CRSF end, it is necessary to ensure that their power consumption is extremely low.

2. Safety Advice: Based on other similar designs (such as the expansion function of the built-in sensor), it is recommended that the current drawn from the CRSF port does not exceed 250mA to prevent the receiver from overheating or getting damaged.

3. Risk Warning: If the sensor requires a larger current, it should be independently powered by the main power supply (such as BEC or battery), rather than relying on the CRSF port.

## Libraries

* [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
* [CRSF for Arduino](https://github.com/ZZ-Cat/CRSFforArduino)
* [ezLED](https://github.com/zetavg/arduino-ezLED)

## References

* [How To Optimize GPS Receiver Settings in U-Center To Get More Satellite Locks](https://oscarliang.com/gps-settings-u-center/)