/**
 * Long Term Environmental Monitoring for 
 * artistic projekt "Time out of Present"
 *
 * receives and displays data from lts-monitor2
 * message is a stream of bytes and contains the following values
 *
 *      0     1        2         3   4    5     6        7    8        9    10   11         12        13            14       15
 * date count latitude longitude IAQ cCO2 ebVOC pressure temp humidity PM25 PM10 brightness windspeed winddirection rainfall SD card
 *
 * @author Jürgen Buchinger
 * @version 1.4 19 Sep 2024
 * 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <LoRa.h>
#include <TimeLib.h>

#define OLED_MOSI     10
#define OLED_CLK      8
#define OLED_DC       7
#define OLED_CS       6
#define OLED_RST      9

Adafruit_SH1106G display = Adafruit_SH1106G(128, 64,OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

/** there are three pages of values to display, we swap at each received message */
int page = 1;


/** all the values to receive */
time_t udate;
unsigned long count;
float lat=0, lon=0, iaq_=0, eco2=0, bvoc=0, pressure=0, temp=0, humidity=0, pm25=0, pm10=0, bright=0, wSpeed=0, wDirection=0, wRain=0;
float data[16];
byte Sd;
char dateString[24];

// for display rhythm
unsigned long last = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("LoRa Receiver");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(0, true)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setTextSize(2);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.println("LTS\nRECEIVER\nv1.4");
  display.display();

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    display.clearDisplay();
    display.println("Starting LoRa failed!");
    display.display();
    while (1);
  }
  Serial.println("1\t2\t3\t4\t5\t6\t7\t8\t9\t10\t11\t12\t13\t14\t14");
  last = millis();
}


void loop() {
  // we will parse the packet here
  int packetSize = LoRa.parsePacket();
  if(packetSize) {    // if a packet was received
    byte bArray[4];    // to hold the data for reconversion to float
    float fValue;
    int t = 0;
    // read packet
    while(LoRa.available() >= 4 && t < 16) {
      // we will only receive 4-byte values so we read 4 bytes at once
      for(int i=0; i<4; i++) {
        bArray[i] = LoRa.read();
      }
      // Convert the byte array back to a float
      if(t==0) {  // first value is time
        memcpy(&udate, bArray, 4);
      } else if(t==1) {
        memcpy(&count, bArray, 4);
      } else {
        memcpy(&fValue, bArray, 4);
        data[t-1] = fValue;
        Serial.print(fValue);
        Serial.print("\t");
      }
      t++;
    }
    if(LoRa.available()) {
      Sd = LoRa.read();
      Serial.println(Sd);
    }
    setTime(udate);
    sprintf(dateString, "%04d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());

    Serial.print(count);
    Serial.print(" ");
    Serial.println(dateString);
  }

  if(millis() - last > 5000) {
    last += 5000;
    displayPage();
  }
}


void displayPage() {
  // display the data
  display.clearDisplay();

  display.setCursor(109,0);
  display.setTextSize(1);
  display.print(page);
  display.print("/4");

  display.setCursor(0, 0);
  display.print(count);
  display.print(" (");
  if(Sd == 1) {
    display.print("SD 1");
  } else if(Sd == 2) {
    display.print("SD 2");
  } else {
    display.print("SD off");
  }
  display.println(")");

  display.println(dateString);

  display.setTextSize(2);
  display.setCursor(0,18);
  if(page == 1) {
    display.print(data[7]);
    display.print(" ");
    display.setTextSize(1);
    display.print("o");
    display.setTextSize(2);
    display.println("C");
    display.print(data[8]);
    display.println(" %Hg");
    display.print(data[4]);
    display.print(" CO");
    display.setTextSize(1);
    display.print("2");
    page++;
  } else if(page == 2) {
    display.print(data[9]);
    display.print(" PM");
    display.setTextSize(1);
    display.print("25");
    display.setTextSize(2);
    display.println();
    display.print(data[10]);
    display.print(" PM");
    display.setTextSize(1);
    display.print("10");
    display.setTextSize(2);
    display.println();      
    display.print(data[5]);
    display.println(" eBVoc");
    page++;
  } else if(page == 3) {
    display.print(data[6]);
    display.println("Pa");
    display.print(data[11]);
    display.println(" lx");
    display.print(data[3]);
    display.println(" IAQ");
    page++;
  } else if(page == 4) {
    display.print(data[12]);
    display.println(" km/h");
    display.print(data[13]);
    display.setTextSize(1);
    display.print("o");
    display.setTextSize(2);
    display.println();
    display.print(data[14]);
    display.println(" mm");
    page = 1;
  }
  display.display();
}