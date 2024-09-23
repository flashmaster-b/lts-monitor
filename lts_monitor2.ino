/**
 * Long Term Environmental Monitoring for 
 * artistic projekt "Time out of Present"
 *
 * Reads environmental data and saves to SD card.
 * 
 *
 * @author Jürgen Buchinger
 * @version 4.1 19 Sep 2024
 * 
 */

#include <Arduino.h>        // required for SERCOM UART port
#include "wiring_private.h" // required for SERCOM UART port
#include <Adafruit_GPS.h>  
#include "bsec.h"           // BME680 gas sensor
#include "SdsDustSensor.h"
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <TimeLib.h>


/**
 * we use two card slots so we can swap on the fly
 */
const int chipSelect1 = 6;
const int chipSelect2 = 7;
int currentCard = chipSelect1;
byte SDOn = 0;


/** 
 * LEDs will show which card is currently being written to
 */
#define LED_C1  5
#define LED_C2  4


/** 
 * the light sensor analog in
 * the pins for the raincounter and 
 * windspeed, will be used as interrupts
 */
#define LIGHT_SENSOR_PIN  A6
#define WINDSPEED_PIN     0
#define RAINWATER_PIN     1
#define WINDDIRECTION_PIN A5


/**
  * We need two UART connections, one for the dust sensor, and one for the GPS-shield
  * so we have to make use of the MKR's SERCOM  thingy to define a additional serial 
  * connection Serial3 on pin 2 & 3 that we will use for the GPS shield
  */
Uart Serial3 (&sercom0, 3, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM0_Handler() {
  Serial3.IrqHandler();
}
#define GPSSerial Serial3
Adafruit_GPS GPS(&GPSSerial);


/** The BME680 gas sensor */
Bsec iaqSensor;


/** the SDS011 dust sensor */
SdsDustSensor sds(Serial1); 


/**
  * we will set this to true if a serial connection is
  * present and will print more information to Serial
  */
bool verbose = false;


/**
 * we will save all data to global variables so we can write and 
 * read them out asynchronous
 */
String datepos = "datetime,fix,fixquality,lat,lon";
String datepos_LoRa ="datetime,fix,fixquality";
String iaq = "IAQ,IAQaccuracy,StaticIAQ,CO2equivalent,bVOCequivalent,pressure,gasOhm,temp,humidity,gasPercentage";
String dust = "PM25,PM10";
String brightness = "brightness";
String windspeed = "windspeed";
String winddirection ="wind direction";
String rainfall = "rainfall";
const String header = "num,"+datepos+","+iaq+","+dust+","+brightness+","+windspeed+","+winddirection+","+rainfall;   // header will be inserted on top of each file and will not change
time_t udate;
float lat=0, lon=0, iaq_=0, eco2=0, bvoc=0, pressure=0, temp=0, humidity=0, pm25=0, pm10=0, bright=0, wSpeed=0, wDirection=0, wRain=0;


/** for keeping track of time, all in ms */
unsigned long timepassed;
int cycletime = 10000;
int dustCycle = 60000;
unsigned long lastDustOn, lastDustOff;
unsigned long count = 0;    // counts the lines of data
int page = 0;     // we have to split LoRa data in pages, otherwise its too long


/** for average calculation and other calculations */
float accBrightness = 0;
int numBrightness = 0;
volatile int windCount=0, rainCount=0;

// this is for the calculation of the wind vane direction from the value of the voltage divider,
// we get these values from the datasheet
const int vaneValues[16] = { 786, 406, 461, 84, 93, 66, 185, 127, 287, 245, 630, 599, 945, 828, 887, 703 };


void setup() {
  Serial.begin(115200);
  delay(3000);
  if(Serial) {
    verbose = true;
    Serial.println("TIME OUT OF PRESENT v3.1");
    Serial.println("Long term environmental monitoring"); 
  }

  pinMode(LED_C1, OUTPUT);
  pinMode(LED_C2, OUTPUT);
  
  // attach the wind and rainwater pins to interruts on falling flank
  pinMode(WINDSPEED_PIN, INPUT_PULLUP);
  pinMode(RAINWATER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WINDSPEED_PIN), wind, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAINWATER_PIN), rain, FALLING);

  analogWrite(LED_C1, 2);
  analogWrite(LED_C2, 0);
  delay(300);
  analogWrite(LED_C1, 0);
  analogWrite(LED_C2, 2);
  delay(300);
  analogWrite(LED_C1, 2);
  analogWrite(LED_C2, 0);
  delay(300);
  analogWrite(LED_C1, 0);
  analogWrite(LED_C2, 2);
  delay(300);
  analogWrite(LED_C1, 2);
  analogWrite(LED_C2, 0);
  delay(300);
  analogWrite(LED_C1, 0);
  delay(300);
  analogWrite(LED_C1, 2);
  analogWrite(LED_C2, 2);


  /**
   * initialize SD card
   */
  initSD();


  /**
   * initialise LoRa communication
   * we will broadcast data over LoRa 
   * for monitoring purposes
   */
  if(verbose) Serial.println("Initialising LoRa...");
  if(!LoRa.begin(868E6)) {
    onError(3, "Starting LoRa failed!");
  } else {
    onError(0,"done.");
  }

  /**
   * Set up GPS 
   * again we need to set up the additional UART functinoality 
   * on pins 2 and 3
   */
  pinPeripheral(2, PIO_SERCOM);
  pinPeripheral(3, PIO_SERCOM);

  /**
   * set up GPS shield with default baud rate for Adafruit MTK GPS's
   */
  GPS.begin(9600);
  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  /**
   * set up BME680 gas sensor
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
   * set up SDS011 dust sensor an Serial1 with 9600 baud
   * we will use a working period of 15s each minute
   */
  sds.begin();
  onError(0, sds.queryFirmwareVersion().toString()); // prints firmware version
  onError(0, sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode

  // lets go!
  onError(0, "starting");
  onError(0, header);
  timepassed = millis();
  lastDustOff = millis();
  lastDustOn = millis();
  analogWrite(LED_C1, 0);
  analogWrite(LED_C2, 0);
}


void loop() {
  /**
   * Light sensor reading
   * Data: brightness
   */
  accBrightness += analogRead(LIGHT_SENSOR_PIN);
  numBrightness++;

  /**
   * GPS reading
   * Data: datetime,fix,fixquality,latitude,longitude
   */
  char c = GPS.read();
  // if a sentence is received, we can check the checksum and parse it
  if(GPS.newNMEAreceived()) {
    if(GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
      datepos = parseDataFromGPS();
    } else {
      onError(2, "Could not parse NMEA messsage");
    }
  }

  /**
   * BME Gas Sensor reading
   * Data: IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, pressure [hPa], gas [Ohm], comp temp[°C], comp humidity [%], gas percentage
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
  if(lastDustOn > lastDustOff) {        // sensor should be on
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
  
  /**
   * each cycle time we save data to SD card, send it via LoRa
   * and print if verbose and also reset the error led
   */
  if(millis() - timepassed > cycletime) {
    timepassed+=cycletime;

    digitalWrite(LED_BUILTIN, LOW);

    // calculate average of brightness
    bright = accBrightness/numBrightness;
    accBrightness=0;
    numBrightness=0;
 
    // read the value from the vind vane voltage divider and
    // calculate the direction in degrees
    int dir = analogRead(WINDDIRECTION_PIN);
    int smallestDifference = abs(dir - vaneValues[0]);
    wDirection = 0;
    for (int i = 1; i < 16; i++) {
        int difference = abs(dir - vaneValues[i]);
        if (difference < smallestDifference) {
            smallestDifference = difference;
            wDirection = i * 22.5;
        }
    }

    // calculate windspeed. As per datasheet, a wind speed of 2.4km/h causes 
    // the switch to close once per second, so v [km/h] = count / sec * 2.4
    // somehow the contact closes twice for each rotation so we divide by two
    wSpeed = (windCount / (cycletime/1000.0)) * 2.4 / 2;
    windCount = 0;

    // calculate rainfall amount, according to datasheet, 1 contact equals
    // 0.2794 mm of rainfall
    // also here the contact closes twice, so / 2 it is
    wRain = rainCount * 0.2794 / 2;
    rainCount = 0;

    writeDataToSD();
    sendData();
    if(verbose) {
      Serial.print(count);
      Serial.print(",");
      Serial.print(datepos);
      Serial.print(",");
      Serial.print(iaq);
      Serial.print(",");
      Serial.print(dust);
      Serial.print(",");
      Serial.print(bright);
      Serial.print(",");
      Serial.print(String(wSpeed));
      Serial.print(",");
      Serial.print(String(wDirection));
      Serial.print(",");
      Serial.print(String(wRain));
      Serial.print(",");
      Serial.println(SDOn);
    }
  }
}

/**
 * parses the data from GPS sensor and returns it
 */
String parseDataFromGPS() {
  tmElements_t tm;
  tm.Year = 2000 + GPS.year - 1970;
  tm.Month = GPS.month;
  tm.Day = GPS.day;
  tm.Hour = GPS.hour;
  tm.Minute = GPS.minute;
  tm.Second = GPS.seconds;
  udate = makeTime(tm);
  lat = GPS.latitude/100.;
  lon = GPS.longitude/100.;
  char datetime[42];
  sprintf(datetime, "20%02d-%02d-%02dT%02d:%02d:%02d,%d,%d,%f,%f", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.fix, GPS.fixquality, GPS.latitude/100., GPS.longitude/100.);
  return datetime;
}


/**
 * reads and parses the data from BME680 gas sensor
 * 
 * @return A comma separated string with all the values read from gas sensor
 */
String parseIAQ() {
  String output = "";
  output += String(iaqSensor.iaq);
  output += "," + String(iaqSensor.iaqAccuracy);
  output += "," + String(iaqSensor.staticIaq);
  output += "," + String(iaqSensor.co2Equivalent);
  output += "," + String(iaqSensor.breathVocEquivalent);
  output += "," + String(iaqSensor.pressure);
  output += "," + String(iaqSensor.gasResistance);
  output += "," + String(iaqSensor.temperature);
  output += "," + String(iaqSensor.humidity);
  output += "," + String(iaqSensor.gasPercentage,6);
  iaq_ = iaqSensor.iaq;
  eco2 = iaqSensor.co2Equivalent;
  bvoc = iaqSensor.breathVocEquivalent;
  pressure = iaqSensor.pressure;
  temp = iaqSensor.temperature;
  humidity = iaqSensor.humidity;
  return output;
}


/**
 * reads and parses data from dust sensor 
 * 
 * @return A comma separated string with all the values read from gas sensor
 */
String parseDust() {
  PmResult pm = sds.queryPm();
  pm25 = pm.pm25;
  pm10 = pm.pm10;
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
 *
 * @param level Error level: 3 - error, 2 - warning, below is information only
 * @param err   The error message
 */
void onError(int level, String err) {
  if(verbose) {
    if(level>2) {
      Serial.print("ERROR: ");
      digitalWrite(LED_BUILTIN, HIGH);
    } else if(level > 0) {
      Serial.print("WARNING: ");
    }
    Serial.println(err);
  }
}


/**
 * writes a dataset to SD card
 * uses the global variables we set for data to write
 * writes to currentCard
 */
void writeDataToSD() {
  // we are setting this high during writing to ensure noone removes disk while writing to it
  int led = currentCard == chipSelect1 ? LED_C1 : LED_C2;
  analogWrite(led, 2);

  char filename[14];
  sprintf(filename, "20%02d%02d%02d.txt", GPS.year, GPS.month, GPS.day);
  String num;

  if(SD.exists(filename)) {   
    num = String(count);
  } else {                    // if we make a new file, include header
    num = header+"\n"+String(count);
  }

  File dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(num);
    dataFile.print(",");
    dataFile.print(datepos);
    dataFile.print(",");
    dataFile.print(iaq);
    dataFile.print(",");
    dataFile.print(dust);
    dataFile.print(",");
    dataFile.print(String(bright));
    dataFile.print(",");
    dataFile.print(String(wSpeed));
    dataFile.print(",");
    dataFile.print(String(wDirection));
    dataFile.print(",");
    dataFile.println(String(wRain,4));
    dataFile.close();
    count++;
    SDOn = currentCard == chipSelect1 ? 1 : 2;
    analogWrite(led, 0);
  } else {                  // if not, produce error
    analogWrite(led, 64);   // bright light on the respective SD card slot = error
    onError(3, "Error opening file: '"+String(filename)+"'");
    onError(2, "Trying to re-initialize...");
    currentCard = currentCard == chipSelect1 ? chipSelect2 : chipSelect1; // change card slots
    SDOn = 0;
    // we will initalize the other card
    // and if it succeeds, we try to write again
    if(initSD()) {
      writeDataToSD();
      analogWrite(led, 0);  // reset error led because all is good now
    } else {
      onError(3, "Initialization of other SD failed!");
    }
  }
}


/**
 * initialize SD Card reader
 *
 * @return true on successful initialization, false on error
 */
bool initSD() {
  // turn the not active card led off no matter whether its working or not
  int led = currentCard == chipSelect1 ? LED_C2 : LED_C1;
  analogWrite(led, 0);
  // turn the active card led on
  led = currentCard == chipSelect1 ? LED_C1 : LED_C2;
  analogWrite(led, 64);
  onError(0, "Initializing SD card...");
  if(!SD.begin(currentCard)) {
    onError(3, "SD initialization failed.");
    return false;
  }
  onError(0, "initialization done.");
  analogWrite(led, 0);
  return true;
}


/**
 * send data over LoRa, uses global variables to send
 * we send as floats rather than text to save bandwidth
 */
void sendData() {
  // first we convert the numbers to bytes
  // we are sending 4 byte values (float and unsigned long)
  byte bCount[4];
  ulongToBytes(count, bCount);
  byte bDate[4];
  ulongToBytes(udate, bDate);
  byte bLat[4];
  floatToBytes(lat, bLat);
  byte bLon[4];
  floatToBytes(lon, bLon);
  byte bIaq[4];
  floatToBytes(iaq_, bIaq);
  byte bEco2[4];
  floatToBytes(eco2, bEco2);
  byte bBvoc[4];
  floatToBytes(bvoc, bBvoc);
  byte bPressure[4];
  floatToBytes(pressure, bPressure);
  byte bTemp[4];
  floatToBytes(temp, bTemp);
  byte bHumidity[4];
  floatToBytes(humidity, bHumidity);
  byte bPm25[4];
  floatToBytes(pm25, bPm25);
  byte bPm10[4];
  floatToBytes(pm10, bPm10);
  byte bBright[4];
  floatToBytes(bright, bBright);
  byte bWSpeed[4];
  floatToBytes(wSpeed, bWSpeed);
  byte bWDirection[4];
  floatToBytes(wDirection, bWDirection);
  byte bRain[4];
  floatToBytes(wRain, bRain);


  // send packet
  LoRa.beginPacket();
  LoRa.write(bDate, 4);
  LoRa.write(bCount, 4);
  LoRa.write(bLat, 4);
  LoRa.write(bLon, 4);
  LoRa.write(bIaq, 4);
  LoRa.write(bEco2, 4);
  LoRa.write(bBvoc, 4);
  LoRa.write(bPressure, 4);
  LoRa.write(bTemp, 4);
  LoRa.write(bHumidity, 4);
  LoRa.write(bPm25, 4);
  LoRa.write(bPm10, 4);
  LoRa.write(bBright, 4);
  LoRa.write(bWSpeed, 4);
  LoRa.write(bWDirection, 4);
  LoRa.write(bRain, 4);
  LoRa.write(SDOn);       // this is already saved as byte
  LoRa.endPacket();
}


/**
 * converts float number to byte array
 */
void floatToBytes(float value, byte* byteArray) {
  memcpy(byteArray, &value, 4); // Copy the float value into the byte array
}


/**
 * converts unseigned int to byte array
 */
void ulongToBytes(unsigned long value, byte* byteArray) {
  memcpy(byteArray, &value, 4); // Copy the value into the byte array
}


/**
 * converts unseigned int to byte array
 */
void longToBytes(long value, byte* byteArray) {
  memcpy(byteArray, &value, 4); // Copy the value into the byte array
}

/**
 * converts a java timestamp string to a unix timestamp
 */
unsigned long timeToSeconds() {

}


/**
 * interrupt service routine for anemometer
 */
void wind() {
  windCount++;
}


/**
 * interrupt service routine for rain gauge
 */
void rain() {
  rainCount++;
}