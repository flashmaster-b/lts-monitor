/**
 * Long Term Environmental Monitoring for 
 * artistic projekt "Time out of Present"
 *
 * receives and displays data from lts-monitor2
 * message is as follows
 *
 * 0     1        2   3          4   5           6         7             8              9        10     11   12       13            14   15   16         17
 * count,datetime,fix,fixquality,IAQ,IAQaccuracy,StaticIAQ,CO2equivalent,bVOCequivalent,pressure,gasOhm,temp,humidity,gasPercentage,PM25,PM10,brightness,SDOn
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

#define OLED_MOSI     10
#define OLED_CLK      8
#define OLED_DC       7
#define OLED_CS       6
#define OLED_RST      9

Adafruit_SH1106G display = Adafruit_SH1106G(128, 64,OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

int page = 1;

void setup() {
  Serial.begin(9600);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("LoRa Receiver");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(0, true)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setTextSize(2);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.println("LTS\nRECEIVER\nv1.2");
  display.display();

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if(packetSize) {
    String msg[19];
    for(int i=0; i<19; i++) {
      msg[i] = "";
    }
    int i=0;
    // read packet
    while (LoRa.available()) {
      char a = (char)LoRa.read();
      if(a == ',') {
        i++;
      } else if(a == 'T') {
        msg[i] += ' ';
      } else {
        msg[i] += a;
      }
    }

    // received a packet
    display.clearDisplay();
    display.setCursor(109,0);
    display.setTextSize(1);
    display.print(page);
    display.print("/3");
    display.setCursor(0, 0);
    display.print(msg[0] + " (");
    if(msg[17] == "1") {
      display.print("SD 1");
    } else if(msg[17] == "2") {
      display.print("SD 2");
    } else {
      display.print("SD off");
    }
    display.println(")");
    display.println(msg[1]);
    display.setTextSize(2);
    display.setCursor(0,18);
    if(page == 1) {
      display.print(msg[11]+" ");
      display.setTextSize(1);
      display.print("o");
      display.setTextSize(2);
      display.println("C");
      display.println(msg[12]+" %Hg");
      display.print(msg[7]+ " CO");
      display.setTextSize(1);
      display.print("2");
      page++;
    } else if(page == 2) {
      display.print(msg[14]+" PM");
      display.setTextSize(1);
      display.print("25");
      display.setTextSize(2);
      display.println();
      display.print(msg[15]+" PM");
      display.setTextSize(1);
      display.print("10");
      display.setTextSize(2);
      display.println();      
      display.println(msg[4]+" IAQ");
      page++;
    } else if(page == 3) {
      display.println(msg[9]+"Pa");
      display.println(msg[16]+" lx");
      page=1;
    }
    display.display();
  }
}
