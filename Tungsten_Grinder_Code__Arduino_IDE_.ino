// Program for automation of tungsten grinder machine project

#include <AccelStepper.h>     // Library for Steppers
#include <Adafruit_GFX.h>     // Library for Screen
#include <Adafruit_SSD1306.h> // Library for Screen
#include <Wire.h>             // Library for I2C
#include <Servo.h>            // Library for Servos
#include "Adafruit_seesaw.h"  // Library for dial
#include <seesaw_neopixel.h>  // Library for neopixel
#include <SPI.h>              // Library for screen

#define sliderStepperDir 9                // Pin for slider stepper direction
#define sliderStepperPul 8                // Pin for slider stepper pulse
#define armStepperDir    7                // Pin for arm stepper direction
#define armStepperPul    6                // Pin for arm stepper pulse
#define hopperStepperDir 5                // Pin for hopper stepper direction
#define hopperStepperPul 4                // Pin for hopper stepper pulse
#define chuckStepperDir  3                // Pin for chuck stepper direction
#define chuckStepperPul  2                // Pin for chuck stepper pulse

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


int32_t stepperMinPulse = 10;
int32_t stepperAcceleration = 500;
int32_t homingStepperMaxSpeed = 100;
int32_t stepperHomingSpeed = 100;
int32_t stepperMaxSpeed = 500;

int32_t numOfTimes = 0;                 // Number of times user wants machine to run 
int32_t unloadingServoAngle = 165;      // This is the position where it meets up with the hopper
int32_t loadingServoAngle = 20;         // This is the position where it meets up with the chuck
int32_t clampServoOpen = 0;             // angle for when the clamp is open
int32_t clampServoClosed = 90;          // angle for when the clamp is closed
int32_t stepsPerRev = 800;              // The amount of steps it takes to do a full rotation
int32_t slideReloadingPosition = -(37.25 * stepsPerRev);  // NEED TO CHANGE The reloading position of the slider
int32_t currSize;                       // The size that the user wants to run
int32_t limitSwitchFlag = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);             // Creates display object
AccelStepper sliderStepper(AccelStepper::DRIVER, sliderStepperPul, sliderStepperDir); // Creates slider stepper object
AccelStepper armStepper(AccelStepper::DRIVER, armStepperPul, armStepperDir);          // Creates arm stepper object
AccelStepper hopperStepper(AccelStepper::DRIVER, hopperStepperPul, hopperStepperDir); // Creates hopper stepper object
AccelStepper chuckStepper(AccelStepper::DRIVER, chuckStepperPul, chuckStepperDir);    // Creates chuck stepper object
Adafruit_seesaw encoder;                                                              // Creates encoder object
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);        // Creates neopixel object

int32_t encoder_position;                           // Encoder position
String sizes[4] = {"1/8", "3/32", "1/16", ".040"};  // The different diameters of tungsten used for both display and hopper position

//int stepsForHopper[4] = {0, 66, 123, 171};          // Steps the hopper stepper need to take for each size {1/8", 3/32", 1/16", .040"}
int stepsForHopper[4] = {171, 123, 66, 0};          // Steps the hopper stepper need to take for each size {1/8", 3/32", 1/16", .040"}

Servo rotatingArmServo; // Creates rotating arm servo object
Servo clampingServo;    // Creates clamping arm servo object

void setup() {
  Serial.begin(9600);
  pinMode(homingSliderLimitSwitch, INPUT_PULLUP);         // Sets the slider homing limit switch to an INPUT
  pinMode(homingArmLimitSwitch, INPUT_PULLUP);            // Sets the arm homing limit switch to an INPUT
  pinMode(homingHopperLimitSwitch, INPUT_PULLUP);         // Sets the hopper homing limit switch to an INPUT
  pinMode(sliderHardLimitSwitch, INPUT_PULLUP);           // Sets the slider hard stop limit switch to an INPUT
  pinMode(rotatingArmLoadedLimitSwitch, INPUT_PULLUP);    // Sets the rotating arm loaded limit switch to an INPUT
  pinMode(rotatingArmUnloadedLimitSwitch, INPUT_PULLUP);  // Sets the rotating arm unloaded limit switch to an INPUT
  
  rotatingArmServo.attach(rotatingArmServoPin); // Initializes rotating arm servo to the correct pin
  clampingServo.attach(clampingServoPin);       // Initializes clamping servo to the correct pin
  
  sliderStepper.setMinPulseWidth(stepperMinPulse);    // Sets the pulse width of the slider stepper motor, this is needed because the Teensy is too fast
  sliderStepper.setMaxSpeed(1200);   // Sets max speed of the slider stepper motor (NEED TO FIND BEST VALUE)
  sliderStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the slider stepper motor (NEED TO FIND BEST VALUE)
  armStepper.setMinPulseWidth(stepperMinPulse);       // Sets the pulse width of the arm stepper motor, this is needed because the Teensy is too fast
  armStepper.setMaxSpeed(homingStepperMaxSpeed);      // Sets max speed of the arm stepper motor (NEED TO FIND BEST VALUE)
  armStepper.setAcceleration(stepperAcceleration);    // Sets max acceleration of the arm stepper motor (NEED TO FIND BEST VALUE)
  hopperStepper.setMinPulseWidth(stepperMinPulse);    // Sets the pulse width of the hopper stepper motor, this is needed because the Teensy is too fast
  hopperStepper.setMaxSpeed(homingStepperMaxSpeed);   // Sets max speed of the hopper stepper motor (NEED TO FIND BEST VALUE)
  hopperStepper.setAcceleration(stepperAcceleration); // Sets max acceleration of the hopper stepper motor (NEED TO FIND BEST VALUE)
  chuckStepper.setMinPulseWidth(stepperMinPulse);     // Sets the pulse width of the chuck stepper motor, this is needed because the Teensy is too fast
  chuckStepper.setMaxSpeed(homingStepperMaxSpeed);    // Sets max speed of the chuck stepper motor (NEED TO FIND BEST VALUE)
  chuckStepper.setAcceleration(stepperAcceleration);  // Sets max acceleration of the chuck stepper motor (NEED TO FIND BEST VALUE)

  // Move hopper CCW (CHECK) unit it hits homing limit switch
  //elapsedMillis hopperTime;
  limitSwitchFlag = 0;
  while (limitSwitchFlag == 0 /*|| hopperTime <= 3000*/) { // CHANGE HOPPER TIME VALUE
    hopperStepper.move(-1);
    hopperStepper.setSpeed(stepperHomingSpeed);
    hopperStepper.runSpeedToPosition();
    delay(20);
    if(digitalRead(homingHopperLimitSwitch) == LOW) {
      limitSwitchFlag = 1;
    }
  }
  
  delay(5000);
  // Position of hopper will be on the 1/8" slot and will be on position 0
  hopperStepper.setCurrentPosition(0);
  limitSwitchFlag = 0; // Set the limit switch flag back to 0 for future homings

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

  // Move hopper to the currect size, currSize = 0 move 0 steps, currSize = 1 move 29.75 degrees 66 steps, currSize = 2 move 55.45 degrees 123 steps, currSize = 3 move 77.23 degrees 171-172 steps 
  hopperStepper.moveTo(400 + stepsForHopper[currSize]);
  hopperStepper.runToPosition();
  delay(500);
  // Note: don't want to set the hopper stepper to 0 becuase we can just tell the stepper to move 800-stepsForHopper[currSize]

  // Move slide back until it hits homing limit switch
  //elapsedMillis slideTime; 
  while (limitSwitchFlag == 0 /*|| slideTime <= 5000*/) { // CHANGE SLIDE TIME VALUE
    sliderStepper.move(1);
    sliderStepper.setSpeed((float)800);
    sliderStepper.runSpeedToPosition();
    delay(13);
    if (digitalRead(homingSliderLimitSwitch) == LOW) {
      limitSwitchFlag = 1;
    }
  }
  delay(500);
  // Test to see if this actually sets the current position to 0
  sliderStepper.setCurrentPosition(0);
  limitSwitchFlag = 0;
  //slideHomePosition = sliderStepper.currentPosition();
  // Move arm CCW until it hits homing limit switch
  //elapsedMillis armTime;
  while (limitSwitchFlag == 0 /*|| armTime <= 2000*/) { // CHANGE ARM TIME VALUE
    armStepper.move(1);
    armStepper.setSpeed(stepperHomingSpeed);
    armStepper.runSpeedToPosition();
    delay(20);
    if(digitalRead(homingArmLimitSwitch) == LOW) {
      limitSwitchFlag = 1;
    }
  }
  delay(500);
  armStepper.setCurrentPosition(0);
  limitSwitchFlag = 0;

  clampingServo.write(clampServoOpen);
  
  // Move rotating arm until it hits unloaded limit switch
  rotatingArmServo.write(unloadingServoAngle); 
}

void loop() {
  sliderStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the slider stepper motor (NEED TO FIND BEST VALUE)
  armStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the arm stepper motor (NEED TO FIND BEST VALUE)
  hopperStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the hopper stepper motor (NEED TO FIND BEST VALUE)
  chuckStepper.setMaxSpeed(stepperMaxSpeed); // Sets max speed of the chuck stepper motor (NEED TO FIND BEST VALUE)
  int i;
  for (i = 0; i < numOfTimes; i++) {
    hopperLoading(i); // Done Function to load the catch basin from the hopper (if i is 0 it will pass the step of rotating up to the hopper)
    chuckLoadingAndUnloading(1, 1); // Change values to make sure it is spinning in the right direction
    sharpenTungsten(); // Done
    rotatingSpindelLoadAndUnload(0); //Done 
  }
  while (1);
}

void sharpenTungsten() {
  // Move arm to the down position
  armStepper.moveTo(-200);
  armStepper.runToPosition();
  
  // Move slider forward until it hits the sliderHardLimitSwitch and add in a timer so things dont break
  //elapsedMillis sliderHardTime;
  limitSwitchFlag = 0;
  while (limitSwitchFlag == 0 /*|| sliderHardTime <= 5000*/) { // CHANGE SLIDER HARD TIME VALUE
    sliderStepper.move(-1);
    sliderStepper.setSpeed((long)800);
    sliderStepper.runSpeedToPosition();
    delay(20);
    if(digitalRead(sliderHardLimitSwitch) == LOW) {
      limitSwitchFlag = 1;
    }
  }

  limitSwitchFlag = 0;
  // Spin chuck clockwise for 50 rotations
  chuckStepper.moveTo(50 * 800);
  chuckStepper.runToPosition();
  
  // Move slider back 23 rotations (this is about 4.5in)
  sliderStepper.moveTo(sliderStepper.currentPosition() + (23 * 800));
  sliderStepper.runToPosition();
  
  // Rotate arm 90 degrees back up
  armStepper.moveTo(0);
  armStepper.runToPosition();
}

void chuckLoadingAndUnloading(int directionToMove, int load) {
  // Move arm forward for a certain amount of time or length (NEEDS TO BE CALCULATED)
  //sliderStepper.moveTo(slideReloadingPosition); //NEED to calculate the slideReloadingPosition
  //sliderStepper.runToPosition();
  sliderStepper.runToNewPosition((long)slideReloadingPosition);

  // Rotate chuck stepper to tighten/untighten chuck (direction)
  chuckStepper.moveTo(directionToMove * 1600);
  chuckStepper.runToPosition();

  sliderStepper.moveTo(0);
  sliderStepper.runToPosition();

  // Rotate the 
  rotatingSpindelLoadAndUnload(load);

  sliderStepper.moveTo(slideReloadingPosition);
  sliderStepper.runToPosition();
  
  // If direction is equal to (-1 or 1 depending on what direction is what) rotate clamping servo so it releases tungsten
    chuckStepper.moveTo(-directionToMove * 1600);
    clampingServo.write(clampServoOpen);

  // Move slider back
  sliderStepper.moveTo(sliderStepper.currentPosition() + (800*23)); // This is based on 800 steps to do a full rotation, might need to change
  sliderStepper.runToPosition();
}

void rotatingSpindelLoadAndUnload(int load) {
  if (load) {
    // Move rotating servo to the load position
    rotatingArmServo.write(loadingServoAngle);
  } else {
    // Rotate rotating arm servo until it hits the rotatingArmUnloadedLimitSwitch
    rotatingArmServo.write(unloadingServoAngle);
  }
}

void hopperLoading(int currentReloads) {
  // Move hopper stepper so that the desired size slot is pointing towards the top (Should be 180 degrees up)
  if (currentReloads != 0) {
    hopperStepper.moveTo(hopperStepper.currentPosition() + 400);
    hopperStepper.runToPosition();
  }

  // Move hopper stepper 180 degrees so that the tungsten will fall into the catch basin
  hopperStepper.moveTo(hopperStepper.currentPosition() - 400); 
  hopperStepper.runToPosition();
  // Move clamping servo to just over the straight up and down position (clamping tungsten)
  clampingServo.write(clampServoClosed);
}
