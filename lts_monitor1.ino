/**
 * Long Term Environmental Monitoring for 
 * artistic projekt "Time out of Present"
 *
 * Reads environmental data and saves to SD card.
 * 
 *
 * @author Jürgen Buchinger
 * @version 1.0 27 Aug 2024
 * 
 */

#include <Arduino.h>        // required for SERCOM UART port
#include "wiring_private.h" // required for SERCOM UART port
#include <Adafruit_GPS.h>  
#include "bsec.h"           // BME680 gas sensor
#include "SdsDustSensor.h"
#include <SPI.h>
#include <SD.h>

/**
 * for SD card
 */
const int chipSelect = SS;


 /* We need two UART connections, so we have to make use of the MKR's SERCOM 
  * thingy to define a additional serial connection Serial3 on pin 2 & 3 
  */
Uart Serial3 (&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM0_Handler() {
  Serial3.IrqHandler();
}
// what's the name of the hardware serial port?
#define GPSSerial Serial3
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// The BME680 gas sensor
Bsec iaqSensor;

// the SDS011 dust sensor
SdsDustSensor sds(Serial1); 

// print all to Serial
bool verbose = false;

/**
 * we will save all data to global variables so we can write them in one place
 * but read them out asynchronous
 */
String datepos = "datetime,fix,fixquality,lat,lon";
String iaq = "IAQ,IAQaccuracy,StaticIAQ,CO2equivalent,bVOCequivalent,pressure,gasOhm,temp,humidity,gasPercentage";
String dust = "PM25,PM10";

/** for keeping track of time */
unsigned long timepassed;
int cycletime = 10000; // 10 s
int dustCycle = 60000;
unsigned int lastDustOn, lastDustOff;

void setup() {
  if(verbose) {
    Serial.begin(115200);
    delay(3000);
    Serial.println("TIME OUT OF PRESENT");
    Serial.println("Long term environmental monitoring");
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  /**
   * initialize SD card
   */
  if(verbose) Serial.print("Initializing SD card...");
  if(!SD.begin(chipSelect)) {
    onError(3, "SD initialization failed.");
  }
  if(verbose) Serial.println("initialization done.");

  /**
   * Set up GPS 
   * Data: datetime,fix,fixquality,lat,lon
   */
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  /**
   * set up BME680
   * Data: IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, pressure [hPa], gas [Ohm], comp temp[°C], comp humidity [%], gas percentage
   */
  iaqSensor.begin(BME68X_I2C_ADDR_HIGH, Wire);
  String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  if(verbose) Serial.println(output);
  checkIaqSensorStatus();
  bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  /**
   * set up SDS011 dust sensor
   * we will use a working period of 15s each minute
   */
  sds.begin(); // this line will begin Serial1 with given baud rate (9600 by default)

  if(verbose) {
    Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
    Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode
    Serial.println("starting");
  }

  // write header
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(datepos);
    dataFile.print(",");
    dataFile.print(iaq);
    dataFile.print(",");
    dataFile.println(dust);
    dataFile.close();
  }

  if(verbose) {
    Serial.print(datepos);
    Serial.print(",");
    Serial.print(iaq);
    Serial.print(",");
    Serial.println(dust);
  }
  timepassed = millis();
  lastDustOff = millis();
  lastDustOn = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  /**
   * GPS reading out below
   */
  char c = GPS.read();
  // if a sentence is received, we can check the checksum and parse it
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      datepos = parseDataFromGPS();
    } else {
      onError(2, "Could not parse NMEA messsage");
    }
  }

  /**
   * BME Sensor reading below
   */
  if (iaqSensor.run()) { // If new data is available
    iaq = parseIAQ();
  } else {
    checkIaqSensorStatus();
  }

  /**
   * Dust sensor will be running the first 15s each minute,
   * then we query it and let it sleep for the remainder of each minute
   */
  if(lastDustOn > lastDustOff) { // sensor should be on
    if(millis() - lastDustOn > 15000) { // query sensor after 15 s and put in sleep mode
      PmResult pm = sds.queryPm();
      if(pm.isOk()) {
        dust = parseDust();
      } else {
        onError(3, "SDS011: " + pm.statusToString());
      }
      WorkingStateResult state = sds.sleep();
      if(state.isWorking()) {
        onError(2,"Problem with sleeping the dust sensor.");
      }
    lastDustOff = lastDustOn+15000;
    }
  } else {              // sensor should be off
    if(millis() - lastDustOff > 45000) {  // after 45 seconds wake up
      sds.wakeup();
      lastDustOn = lastDustOff+45000;
    }
  }
  
  if(millis() - timepassed > cycletime) {
    timepassed+=cycletime;
    digitalWrite(LED_BUILTIN, HIGH);
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(datepos);
      dataFile.print(",");
      dataFile.print(iaq);
      dataFile.print(",");
      dataFile.println(dust);
      dataFile.close();
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      onError(3, "Error opening datalog.txt");
    }
    if(verbose) {
      Serial.print(datepos);
      Serial.print(",");
      Serial.print(iaq);
      Serial.print(",");
      Serial.println(dust);
    }
  }
}

/**
 * parses the data from GPS sensor and returns it
 */
String parseDataFromGPS() {
  char datetime[42];
  sprintf(datetime, "20%02d-%02d-%02dT%02d:%02d:%02d,%d,%d,%f,%f", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.fix, GPS.fixquality, GPS.latitude/100., GPS.longitude/100.);
  return datetime;
}

/**
 * parses the data from BME680 gas sensor
 */
String parseIAQ() {
  String output = "";
  output += String(iaqSensor.iaq);
  output += ", " + String(iaqSensor.iaqAccuracy);
  output += ", " + String(iaqSensor.staticIaq);
  output += ", " + String(iaqSensor.co2Equivalent);
  output += ", " + String(iaqSensor.breathVocEquivalent);
  output += ", " + String(iaqSensor.pressure);
  output += ", " + String(iaqSensor.gasResistance);
  output += ", " + String(iaqSensor.temperature);
  output += ", " + String(iaqSensor.humidity);
  output += ", " + String(iaqSensor.gasPercentage);
  return output;
}

/**
 * parses dust sensor 
 */
String parseDust() {
PmResult pm = sds.queryPm();
  return String(pm.pm25) + "," + String(pm.pm10);
}

/**
 * checks BME680 ges sensor status 
 */
void checkIaqSensorStatus(void) {
  String output;
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      onError(3, output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      onError(2, output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      onError(3, output);
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      onError(2, output);
    }
  }
}

/** 
 * handles errors
 * 3 - error
 * 2 - warning
 */
void onError(int level, String err) {
  if(verbose) {
    if(level>2) {
      Serial.print("ERROR: ");
    } else {
      Serial.print("WARNING: ");
    }
    Serial.println(err);
  }
}