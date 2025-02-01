  // Software Serial Sample for USB Sabertooth Packet Serial
// Copyright (c) 2012-2013 Dimension Engineering LLC
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <USBSabertooth.h>

SoftwareSerial       SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
USBSabertoothSerial  C(SWSerial);             // Use SWSerial as the serial port.
USBSabertooth        ST(C, 128);              // Use address 128.
int pinA=2;
int pinB=3;

float kp=8.5; 
float ki=0;
float kd=0;

float setpoint=2000;
float actualvalue;
float error;
float pre ;
float intrgral;
float derivative;
float output;

void setup()
{
  SWSerial.begin(115200);
  pinMode(pinA,INPUT_PULLUP);
  pinMode(pinB,INPUT_PULLUP);
  Serial.begin(115200);

attachInterrupt(digitalPinToInterrupt(2), encodercode, CHANGE);
}

void loop()
{
  error = setpoint - actualvalue;

  intrgral = intrgral + error;

  derivative = pre - error;
  
  output = kp *error + ki* intrgral + kd* derivative;

  ST.motor(1,-output );

  pre = error;

  Serial.print("  setpoint:");
  Serial.print( setpoint);
  Serial.print("  actualvalue:");
  Serial.print(actualvalue);
  Serial.print("  Error:");
  Serial.print(error);
  Serial.print("  Output:");
  Serial.println(output);

}

void encodercode()
{
  int currentA=digitalRead(pinA);
  int currentB=digitalRead(pinB);

  if(currentA==currentB)
  {
    actualvalue++ ;
  }
  else
  {
    actualvalue-- ;
  }
  
}

