// Bounce.pde
// -*- mode: C++ -*-
//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
//#include <ezButton.h>


// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 7, 6); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
//ezButton limitSwitch(17);

//bool isStopped = false;
//#define MAX_POSITION 0x7FFFFFFF // maximum of position we can set (long type)



void setup()
{
  //Serial.begin(9600);

  // Change these to suit your stepper if you want
  //limitSwitch.setDebounceTime(50);
  //pinMode(17, INPUT_PULLUP);
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(500);
  stepper.setMinPulseWidth(10);
}

void loop()
{
//  while (digitalRead(17)) {
//    stepper.move(1);
//    stepper.setSpeed(25);
//    stepper.runSpeedToPosition();
//  }
//  while(1);
//  delay(2000);
  stepper.moveTo(800);
  stepper.runToPosition();
  while(1);
//  delay(1000);
//  stepper.moveTo(0);
//  stepper.runToPosition();
}
