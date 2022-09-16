// Program for automation of tungsten grinder machine project

#include <AccelStepper.h> // Libary for Steppers
#include <Adafruit_GFX.h> // Libary for Screen
#include <Adafruit_SSD1306.h> // Libary for Screen
#include <Wire.h> // Libary for I2C
#include <Servo.h> // Libary for Servos
#include "Adafruit_seesaw.h" // Libary for dial
#include <seesaw_neopixel.h> // Libary for neopixel
#include <SPI.h>

#define sliderStepperDir 9                // Pin for slider stepper direction
#define sliderStepperPul 8               // Pin for slider stepper pulse
#define armStepperDir    7              // Pin for arm stepper direction
#define armStepperPul    6               // Pin for arm stepper pulse
#define hopperStepperDir 5               // Pin for hopper stepper direction
#define hopperStepperPul 4              // Pin for hopper stepper pulse
#define chuckStepperDir  3               // Pin for chuck stepper direction
#define chuckStepperPul  2               // Pin for chuck stepper pulse

#define rotatingArmServoPin  10           // Defines the pin for the rotating arm servo
#define clampingServoPin     11           // Defines the pin for the clamping servo

#define homingSliderLimitSwitch  40       // Homing switch for slider
#define homingArmLimitSwitch     41       // Homing switch for arm
#define homingHopperLimitSwitch  16       // Homing switch for Hopper
#define sliderHardLimitSwitch    17       // Hard limit switch for slider
#define rotatingArmLoadedLimitSwitch 20   // Limit switch for rotating arm on the side that feeds the chuck
#define rotatingArmUnloadedLimitSwitch 21 // Limit switch for rotating arm on the hopper side

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 32    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C // Address for screen I2C
#define OLED_RESET -1       // Screen reset shares the same pin as reset pin

#define SEESAW_ADDR 0x36
#define ENCODER_SWITCH 24
#define SS_NEOPIX        6


#define stepperMinPulse 10
#define stepperAcceleration 500
#define stepperMaxSpeed 100
#define stepperHomingSpeed 50




int32_t numOfTimes = 0; // Number of times user wants machine to run 
int32_t slideHomePosition = 0;
int32_t armHomePosition = 0;
int32_t hopperHomePosition = 0;
int32_t slideReloadingPosition = 1600; // NEED TO CHANGE
int32_t unloadingServoAngle = 0;
int32_t loadingServoAngle = 180; 
int32_t clampServoOpen = 750; // pulse width for opening
int32_t clampServoClosed = 1925; // pulse width for closing
int32_t stepsPerRev = 800;
int32_t currSize;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Creates display object
AccelStepper sliderStepper(AccelStepper::DRIVER, sliderStepperPul, sliderStepperDir); // Creates slider stepper object
AccelStepper armStepper(AccelStepper::DRIVER, armStepperPul, armStepperDir); // Creates arm stepper object
AccelStepper hopperStepper(AccelStepper::DRIVER, hopperStepperPul, hopperStepperDir); // Creates hopper stepper object
AccelStepper chuckStepper(AccelStepper::DRIVER, chuckStepperPul, chuckStepperDir); // Creates chuck stepper object
Adafruit_seesaw encoder;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t encoder_position;
// This is for the hopper system to figure out what angel to go for each size
String sizes[4] = {"1/8", "3/32", "1/16", ".040"};

int stepsForHopper[4] = {100, 200, 300, 400}; //This will need to be changed
int32_t stepsToTakeForHopper;

Servo rotatingArmServo; // Creates rotating arm servo object
Servo clampingServo; // Creates clamping arm servo object

void setup() {
  Serial.begin(9600);
  pinMode(homingSliderLimitSwitch, INPUT_PULLUP); // Sets the slider homing limit switch to an INPUT
  pinMode(homingArmLimitSwitch, INPUT_PULLUP); // Sets the arm homing limit switch to an INPUT
  pinMode(homingHopperLimitSwitch, INPUT_PULLUP); // Sets the hopper homing limit switch to an INPUT
  pinMode(sliderHardLimitSwitch, INPUT_PULLUP); // Sets the slider hard stop limit switch to an INPUT
  pinMode(rotatingArmLoadedLimitSwitch, INPUT_PULLUP); // Sets the rotating arm loaded limit switch to an INPUT
  pinMode(rotatingArmUnloadedLimitSwitch, INPUT_PULLUP); // Sets the rotating arm unloaded limit switch to an INPUT
  
  rotatingArmServo.attach(rotatingArmServoPin); // Initializes rotating arm servo to the correct pin
  clampingServo.attach(clampingServoPin); // Initializes clamping servo to the correct pin
  
  sliderStepper.setMinPulseWidth(stepperMinPulse);
  sliderStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the slider stepper motor (NEED TO FIND BEST VALUE)
  sliderStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the slider stepper motor (NEED TO FIND BEST VALUE)
  armStepper.setMinPulseWidth(stepperMinPulse);
  armStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the arm stepper motor (NEED TO FIND BEST VALUE)
  armStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the arm stepper motor (NEED TO FIND BEST VALUE)
  hopperStepper.setMinPulseWidth(stepperMinPulse);
  hopperStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the hopper stepper motor (NEED TO FIND BEST VALUE)
  hopperStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the hopper stepper motor (NEED TO FIND BEST VALUE)
  chuckStepper.setMinPulseWidth(stepperMinPulse);
  chuckStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the chuck stepper motor (NEED TO FIND BEST VALUE)
  chuckStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the chuck stepper motor (NEED TO FIND BEST VALUE)

   if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.display();
  
  delay(2000);

  display.clearDisplay();
  Serial.println("cleared display");
  
  Serial.println("Looking for seesaw!");
  
  if (! encoder.begin(SEESAW_ADDR) || ! sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((encoder.getVersion() >> 16) & 0xFFFF);
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
  encoder.pinMode(ENCODER_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = encoder.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  encoder.setGPIOInterrupts((uint32_t)1 << ENCODER_SWITCH, 1);
  encoder.enableEncoderInterrupt();
  
  //  Wait until user scrolls to desired size and pushes in the knob to confirm their choice
  //display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Quantity?"));
  Serial.println("Quantity?");
  display.display();
  while (encoder.digitalRead(ENCODER_SWITCH)) {
    int32_t new_position = encoder.getEncoderPosition();
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
  numOfTimes = encoder_position;
  delay(1000);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Size?"));
  Serial.println("Size?");
  display.display();
  while (encoder.digitalRead(ENCODER_SWITCH)) {
    int32_t new_positions = encoder.getEncoderPosition();
    if (encoder_position != new_positions) {
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.println(F("Size?"));
      if (encoder_position < 0) {
        display.println((sizes[(-encoder_position) % 4]));
        Serial.println((sizes[(-encoder_position) % 4]));
      } else {
        display.println((sizes[encoder_position % 4]));
        Serial.println((sizes[encoder_position % 4]));
      }
      display.display();
      encoder_position = new_positions;
    }
    delay(50);
  }
  if (encoder_position < 0) {
    currSize = (-encoder_position) % 4;
  } else {
    currSize = encoder_position % 4;
  }
  
  
  
  //   Wait until user scrolle to desired quantity and pushed in the knob to confirm their choice
  //   Store in variable
  // Move slide back until it hits homing limit switch
  elapsedMillis slideTime; 
  while (digitalRead(homingSliderLimitSwitch) || slideTime <= 5000) { // CHANGE SLIDE TIME VALUE
    sliderStepper.move(1);
    sliderStepper.setSpeed(stepperHomingSpeed);
    sliderStepper.runSpeedToPosition();
  }
  delay(500);
  // Test to see if this actually sets the current position to 0
  sliderStepper.setCurrentPosition(0);
  //slideHomePosition = sliderStepper.currentPosition();
  // Move arm CCW until it hits homing limit switch
  elapsedMillis armTime;
  while (digitalRead(homingArmLimitSwitch) || armTime <= 2000) { // CHANGE ARM TIME VALUE
    armStepper.move(1);
    armStepper.setSpeed(stepperHomingSpeed);
    armStepper.runSpeedToPosition();
  }
  delay(500);
  armStepper.setCurrentPosition(0);
  //armHomePosition = armStepper.currentPosition();
  // Move hopper CCW (CHECK) unit it hits homing limit switch
  elapsedMillis hopperTime;
  while (digitalRead(homingHopperLimitSwitch) || hopperTime <= 1000) { // CHANGE HOPPER TIME VALUE
    hopperStepper.move(1);
    hopperStepper.setSpeed(stepperHomingSpeed);
    hopperStepper.runSpeedToPosition();
  }
  delay(500);
  hopperStepper.setCurrentPosition(0);

  hopperStepper.moveTo(stepsForHopper[currSize]); //This will need to be changed if it is supposed to be pos or neg
  hopperStepper.runToPosition();
  
  // Move rotating arm until it hits unloaded limit switch
  rotatingArmServo.write(loadingServoAngle); // CHANGE THE VALUE
//  elapsedMillis rotatingArmTime;
//  while (digitalRead(rotatingArmUnloadedLimitSwitch) == HIGH || rotatingArmTime <= 2000) {
//    int i = 91;
//    rotatingArmServo.write(i);
//    i++;
//  }
  clampingServo.write(clampServoOpen);
}

void loop() {
  int i;
  // Move hopper stepper up (NOTE: This will be a different value than normal operation) to desired slot
  chuckLoadingAndUnloading(1); // Change values to make sure it is spinning in the right direction
  rotatingArmLoadAndUnload(1); // 1 means that it is loaded
  chuckLoadingAndUnloading(-1); // Change values to make sure it is spinning in the right direction
  rotatingArmLoadAndUnload(-1); // -1 means that it is unloaded and is going to go back to the hopper
  sharpenTungsten();
  //int numTimes = numOfTimes;
  for (i = 0; i < numOfTimes - 1; i++) {
    hopperLoading();
    chuckLoadingAndUnloading(1); // Change values to make sure it is spinning in the right direction
    rotatingArmLoadAndUnload(1); // 1 means that it is loaded
    chuckLoadingAndUnloading(-1); // Change values to make sure it is spinning in the right direction
    rotatingArmLoadAndUnload(-1); // -1 means that it is unloaded and is going to go back to the hopper
    sharpenTungsten();
  }
  while (1);
}

void sharpenTungsten() {
  // Move arm to the down position
  armStepper.moveTo(200); // CHANGE VALUE (COULD BE NEGATIVE)
  armStepper.runToPosition();
  
  // Move slider forward until it hits the sliderHardLimitSwitch and add in a timer so things dont break
  elapsedMillis sliderHardTime;
  while (digitalRead(sliderHardLimitSwitch) || sliderHardTime <= 5000) { // CHANGE SLIDER HARD TIME VALUE
    sliderStepper.move(1);
    sliderStepper.setSpeed(stepperHomingSpeed);
    sliderStepper.runSpeedToPosition();
  }
  // Spin chuck clockwise for 10 seconds (Steps per rev * ~500 rpm / 60 * 10) = (Steps per rev * 83)
  chuckStepper.moveTo(83 * stepsPerRev);
  chuckStepper.runToPosition();
  
  // Move slider back 
  sliderStepper.moveTo(slideHomePosition);
  sliderStepper.runToPosition();
  
  // Rotate arm 90 degrees back up
  armStepper.moveTo(armHomePosition);
  armStepper.runToPosition();
}

void chuckLoadingAndUnloading(int directionToMove) {
  // Move arm forward for a certain amount of time or length (NEEDS TO BE CALCULATED)
  sliderStepper.moveTo(slideReloadingPosition);
  sliderStepper.runToPosition();

  // Rotate chuck stepper to tighten/untighten chuck (direction)
  chuckStepper.moveTo(directionToMove * 600);
  chuckStepper.runToPosition();
  // If direction is equal to (-1 or 1 depending on what direction is what) rotate clamping servo so it releases tungsten
  if (directionToMove == 1) {
    clampingServo.write(clampServoOpen);
  }
  // Move slider back
  sliderStepper.moveTo(sliderStepper.currentPosition() - 18400); // This is based on 800 steps to do a full rotation, might need to change
  sliderStepper.runToPosition();
}

void rotatingArmLoadAndUnload(int loadOrUnload) {
  if (loadOrUnload) {
    // Move clamping servo to just over the straight up and down position (clamping tungsten)
    clampingServo.write(clampServoClosed);
    // Rotate rotating arm servo until it hits the rotatingArmLoadedLimitSwitch
//    rotatingArmServo.write(90); // Change the value
//    while (digitalRead(rotatingArmLoadedLimitSwitch)) {
//      int i = 89;
//      rotatingArmServo.write(i);
//      i--;
//    }
  rotatingArmServo.write(loadingServoAngle);
  } else {
    // Rotate rotating arm servo until it hits the rotatingArmUnloadedLimitSwitch
    rotatingArmServo.write(unloadingServoAngle);
  }
}

void hopperLoading() {
  // Move hopper stepper so that the desired size slot is pointing towards the top (Should be 180 degrees up)
  hopperStepper.moveTo(hopperStepper.currentPosition() - 400); //NEEDS A VALUE (MAY NEED TO CHANGE VALUE OR SIGN)
  hopperStepper.runToPosition();

  // Move hopper stepper 180 degrees so that the tungsten will fall into the catch basin
  hopperStepper.moveTo(hopperStepper.currentPosition() + 400); //NEEDS A VALUE (MAY NEED TO CHANGE VALUE OR SIGN)
  hopperStepper.runToPosition();
}
