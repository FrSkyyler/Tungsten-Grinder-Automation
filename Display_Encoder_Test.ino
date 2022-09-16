#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define SS_SWITCH        24
#define SS_NEOPIX        6

#define SEESAW_ADDR          0x36

Adafruit_seesaw ss;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t encoder_position;
String sizes[4] = {"1/8", "3/32", "1/16", ".040"};
int32_t quantity;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //while (!Serial) delay(10);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.display();
  
  delay(2000);

  display.clearDisplay();
  Serial.println("cleared display");
  
  Serial.println("Looking for seesaw!");
  
  if (! ss.begin(SEESAW_ADDR) || ! sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version  != 4991){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
  }
  Serial.println("Found Product 4991");

  // set not so bright!
  sspixel.setBrightness(20);
  sspixel.show();
  
  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();

  
}

void loop() {
  // put your main code here, to run repeatedly:
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Quantity?"));
  Serial.println("Quantity?");
  display.display();
  while (ss.digitalRead(SS_SWITCH)) {
    int32_t new_position = ss.getEncoderPosition();
    if (encoder_position != new_position) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.println(F("Quantity?"));
      Serial.println("Quantity?");
      display.println(new_position);
      Serial.println(new_position);
      display.display();
      encoder_position = new_position;      // and save for next round
    }
    delay(20);
  }
  quantity = encoder_position;
  delay(1000);
  //encoder_position = 0;
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Size?"));
  Serial.println("Size?");
  display.display();
  while (ss.digitalRead(SS_SWITCH)) {
    int32_t new_positions = ss.getEncoderPosition();
    if (encoder_position != new_positions) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.println(F("Size?"));
      if (encoder_position < 0) {
        display.println((sizes[(-encoder_position) % 4]));
        Serial.println((sizes[(-encoder_position) % 4]));
        //currSize = sizes[4 + (new_positions % 4)];
      } else {
        display.println((sizes[encoder_position%4]));
        Serial.println((sizes[encoder_position%4]));
        //currSize = sizes[new_positions % 4];
      }
      display.display();
      encoder_position = new_positions;
    }
    delay(50);
  }

  for (int i = quantity; i >= 1; i--) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("quantity"));
    display.print(i);
    display.display();
    delay(1500);
  }

}
