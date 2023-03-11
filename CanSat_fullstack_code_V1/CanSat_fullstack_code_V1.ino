/*
* Authors: Cansat team Beagle 3
* Date: 
* This code will be reading data from :
*     - Adafruit Ultimate GPS
*     - Adafruit BME280 environmetal sensor
*     - DF Robots VELM6075----- radiation sensor
*     - It also utilizes the serial ports for data transmission
*       using the APC220 rf module
*
*  Majority of this code is obtained from the Adafruit Test Libraries
*   for sensors
*
*/

#include <Adafruit_GPS.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <DFRobot_VEML6075.h>


// define GPSSerial Serial1 on Arduino NANO BLE 33
UART GPSSerial(digitalPinToPinName(4), digitalPinToPinName(3), NC, NC); // order of pins is TX RX



// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

#define VEML6075_ADDR   0x10

DFRobot_VEML6075_IIC VEML6075(&Wire, VEML6075_ADDR);  // create object

bool GPS_online = false;
bool BME_online = false;
bool VELM_online = false;

void setup()
{
  Serial1.begin(9600);
  Serial1.println("CanSat Data transmission Version 0!");
  startGPS();
  startBME();
  startVELM();
}

void loop() 
{
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {

    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }  
   // approximately every 1 second or so, print out GPS and sensor readings
  if (millis() - timer > 900) {
    timer = millis(); // reset the timer

    if (GPS_online) {
      valuesGPS();
    } else {
      Serial1.println("~~~~~~~~~~~~~~GPS is OFFLINE!~~~~~~~~~~~~~~~~~~~~");
      startBME();      
    }
   
    if (BME_online) {
      valuesBME280();
    } else {
      Serial1.println("~~~~~~~~~~~~~~BME is OFFLINE!~~~~~~~~~~~~~~~~~~~~");
      startBME();      
    }
    
    if (VELM_online) {
      valuesUV();
    } else {
      Serial1.println("~~~~~~~~~~~~~~VELM is OFFLINE!~~~~~~~~~~~~~~~~~~~~");
      startVELM();
    }
  }
}

void valuesGPS() {
  
  Serial1.print("\nTime: ");
  Serial1.print(GPS.hour, DEC); Serial1.print(':');
  Serial1.print(GPS.minute, DEC); Serial1.print(':');
  Serial1.print(GPS.seconds, DEC); Serial1.print('.');

  Serial1.println(GPS.milliseconds);
  Serial1.print("Date: ");
  Serial1.print(GPS.day, DEC); Serial1.print('/');
  Serial1.print(GPS.month, DEC); Serial1.print("/20");
  Serial1.println(GPS.year, DEC);
  Serial1.print("Fix: "); Serial1.print((int)GPS.fix);
  Serial1.print(" quality: "); Serial1.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial1.print("Location: ");
    Serial1.print(GPS.latitude, 4); Serial1.print(GPS.lat);
    Serial1.print(", ");
    Serial1.print(GPS.longitude, 4); Serial1.println(GPS.lon);
    Serial1.print("Speed (knots): "); Serial1.println(GPS.speed);
    Serial1.print("Angle: "); Serial1.println(GPS.angle);
    Serial1.print("Altitude: "); Serial1.println(GPS.altitude);
    Serial1.print("Satellites: "); Serial1.println((int)GPS.satellites);
    Serial1.print("Antenna status: "); Serial1.println((int)GPS.antenna);
  }
}

void valuesBME280() {
  Serial1.println();

  Serial1.print("Tem:");
  Serial1.print(bme.readTemperature());

  Serial1.print(" Pre:");
  Serial1.print(bme.readPressure() / 100.0F);

  Serial1.print(" ~Alt:");
  Serial1.print(bme.readAltitude(SEALEVELPRESSURE_HPA));

  Serial1.print(" Hmdt:");
  Serial1.print(bme.readHumidity());

  Serial1.println();
}

void valuesUV(){

  Serial1.println();
  Serial1.print("UVAR:");
  Serial1.print(VEML6075.readUvaRaw()); // read UVA raw
  Serial1.print(", UVBR:");
  Serial1.print(VEML6075.readUvbRaw()); // read UVB raw
  Serial1.print(", COMP1R:");
  Serial1.print(VEML6075.readUvComp1Raw()); // read COMP1 raw
  Serial1.print(", COMP2R:");
  Serial1.print(VEML6075.readUvComp2Raw()); // read COMP2 raw
  Serial1.print(", UVA:");
  Serial1.print(VEML6075.getUva(), 2); // get UVA
  Serial1.print(", UVB:");
  Serial1.print(VEML6075.getUvb(), 2); // get UVB
  Serial1.print(", UVIndex:");
  Serial1.print(VEML6075.getUvi(VEML6075.getUva(), VEML6075.getUvb()), 2); // get UV index
  Serial1.println();
}

void startGPS() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if (GPS.begin(9600)) {
    Serial1.println("~~~~~~~~~~~~~~GPS is ONLINE!~~~~~~~~~~~~~~~~~~~~");
     // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    // Ask for firmware version
    Serial1.println("GPS firmeware version: ");
    Serial1.println(GPSSerial.println(PMTK_Q_RELEASE));
    delay(1000); // do we need this?
    GPS_online = true;
  } else {
    GPS_online = false;
  }
}

void startBME() {
  unsigned status;
  status = bme.begin();  
  if (!status) {
      BME_online = false;
  }
  else {
    BME_online = true;
    Serial1.println("~~~~~~~~~~~~~~BME is ONLINE!~~~~~~~~~~~~~~~~~~~~");
  }
}

void startVELM() {
  if (!VEML6075.begin()) {
    VELM_online = false;
  } else {
    VELM_online = true;
    Serial1.println("~~~~~~~~~~~~~~ VEML6075 is ONLINE!~~~~~~~~~~~~~~~~~~~");
  }
  
}
