/**
 * Long Term Environmental Monitoring for 
 * artistic projekt "Time out of Present"
 *
 * receives and displays data from lts-monitor1
 * message is as follows
 *
 * 0     1        2   3          4   5           6         7             8              9        10     11   12       13            14   15   16         17
 * count,datetime,fix,fixquality,IAQ,IAQaccuracy,StaticIAQ,CO2equivalent,bVOCequivalent,pressure,gasOhm,temp,humidity,gasPercentage,PM25,PM10,brightness,SDOn
 *
 * @author Jürgen Buchinger
 * @version 1.2 28 Aug 2024
 * 
 */

#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int page = 1;

void setup() {
  Serial.begin(9600);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("LoRa Receiver");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.setTextSize(2);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
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
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(109,0);
    display.setTextSize(1);
    display.print(page);
    display.print("/3");
    display.setCursor(0, 0);
    display.print(msg[0] + " (");
    if(msg[17] == "1") {
      display.print("SD on");
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
