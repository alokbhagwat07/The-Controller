#include "CytronMotorDriver.h"

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include <Servo.h>

CytronMD motor(PWM_DIR, 9, 10);

SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

SoftwareSerial SWSerial2(NOT_A_PIN, 13); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST2(SWSerial2); // Use SWSerial as the serial port.




Servo myservo;


int throt1;
int throt2;
int throt4;
int throt4L;
int throt4R;
int throt5;
int throt3;
int servo_grab;




int m1;
int m2;
int m3;
int m4;
int ig32;

void setup()
{

  pinMode(7, INPUT);
  pinMode(6, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(3, INPUT);
  pinMode(2, INPUT);

  myservo.attach(12);

  Serial.begin(115200);


  SWSerial.begin(9600);
  SWSerial2.begin(9600);
}

void loop()
{
  throt1 = pulseIn(7, 1);
  throt2 = pulseIn(6, 1);
  throt4 = pulseIn(4, 1);
  throt5 = pulseIn(5, 1);
  throt3 = pulseIn(3, 1);
  servo_grab = pulseIn(2, 1);


  Serial.print("Throt1 : ");
  Serial.print(throt1);
  Serial.print("  Throt2 : ");
  Serial.print(throt2);
  Serial.print("  Throt4 : ");
  Serial.print(throt4);
  Serial.print("  Throt3 : ");
  Serial.print(throt3);
  Serial.print("  Throt5 : ");
  Serial.print(throt5);
  Serial.print("  servo_grab : ");
  Serial.print(servo_grab);


    //throt 1//

  throt1 = map(throt1, 931, 1923, -90, 90);
  if (throt1 > -25 && throt1 < 22) {
    throt1 = 0;
  }

  //throt 2//

  throt2 = map(throt2, 1030, 2025, -110, 110);
  if (throt2 > -26 && throt2 < 10){
     throt2 = 0;
  }

  //throt 4//

  throt4 = map(throt4 , 1055 , 1923, -127, 127 );

  if (throt4 > -100 && throt4 < -10)
  {
    throt4R = throt4;
  }
  if (throt4 > 21 && throt4 < 100)
  {
    throt4L = throt4;
  }
  if (throt4 > -10 && throt4 < 21)
  {
    throt4L = 0;
    throt4R = 0;
    throt4 = 0;
  }

   //throt 5//

  if (throt5 < 1010) {
    throt5 = 0;
  }
  if (throt5 > 1950) {
    throt5 = 1;
  }

  //  ______________________channel for ig32//

  throt3 = map(throt3, 995, 1990, -100, 100);
    //  _____________________________________________//

  //_channel for servo_//

  if (servo_grab < 1010) {
    servo_grab = 0;
  }
  if (servo_grab > 1950) {
    servo_grab = 1;
//    Serial.print(" Hii 1 ");
  }

   //  _____________________________________________//

  if (throt5 == 1) {
    throt4 = 0;
    throt4L = 0;
    throt4R = 0;

    if (throt3 > -30 && throt3 < 20) {
      throt3 = 0;
    }

    if (throt3 > 20) {
      throt3 = 17;           

 Serial.print("   forward");
    }
    if (throt3 < -30) {
      throt3 = -30;
      Serial.print("  backward");

    }

    if (servo_grab == 1) {
      Serial.print(" Hii ON ");
      myservo.write(130);
    }
    if (servo_grab == 0) {
      Serial.print(" Hii OFF ");
      myservo.write(160);
    }

    Serial.print("  throt3 ON");
  }



   if (throt5 == 0) {
    throt3 = 0;
    Serial.print("   throt3 OFF");
  }


//  myservo.write(110); // open = 87  Grab = 110

  //_motor chnnl Assign//

  m1 = -throt1 + throt2 - throt4R;
  m2 = -throt1 + throt2 - throt4R;
  m3 = throt1 + throt2 + throt4L;
  m4 = -throt1 - throt2 - throt4L;
  ig32 = -throt3;
  

  ST.motor(2, m1);    // Motor - 1 = Foront_Right
  ST.motor(1, m2);   // Motor - 2 = Back_Right
  ST2.motor(2, m3);     // Motor - 3 = Back_left
  ST2.motor(1, m4);   // Motor - 4 = Front_Left

  motor.setSpeed(ig32);

 Serial.print("  Throt1 : ");
  Serial.print(throt1);
  Serial.print("  Throt2 : ");
  Serial.print(throt2);
  Serial.print("  Throt4 : ");
  Serial.print(throt4);
  Serial.print("  Throt3 : ");
  Serial.print(throt3);
  Serial.print("  Throt5 : ");
  Serial.print(throt5);
 Serial.print("  servo_grab : ");
 Serial.print(servo_grab);


   Serial.println(" ");

}