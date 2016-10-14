/*
  simpleMovements.ino

 This  sketch simpleMovements shows how they move each servo motor of Braccio

 Created on 18 Nov 2015
 by Andrea Martino

 This example is in the public domain.
 */

#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

Braccio braccio = Braccio(base, shoulder, elbow, wrist_ver, wrist_rot, gripper);

void setup() {
  //Initialization functions and set up the initial position for Braccio
  //All the servo motors will be positioned in the "safety" position:

  // initialization pin Servo motors
  #if defined(ARDUINO_ARCH_SAMD)
    base.attach(11);
    shoulder.attach(7);
    elbow.attach(9);
    wrist_ver.attach(6);
    wrist_rot.attach(8);
    gripper.attach(3);
  #else
    base.attach(11);
    shoulder.attach(10);
    elbow.attach(9);
    wrist_ver.attach(6);
    wrist_rot.attach(5);
    gripper.attach(3);
  #endif

  braccio.home();
}

void loop() {
   /*
   Step Delay: a milliseconds delay between the movement of each servo.  Allowed values from 10 to 30 msec.
   M1=base degrees. Allowed values from 0 to 180 degrees
   M2=shoulder degrees. Allowed values from 15 to 165 degrees
   M3=elbow degrees. Allowed values from 0 to 180 degrees
   M4=wrist vertical degrees. Allowed values from 0 to 180 degrees
   M5=wrist rotation degrees. Allowed values from 0 to 180 degrees
   M6=gripper degrees. Allowed values from 10 to 73 degrees. 10: the toungue is open, 73: the gripper is closed.
  */

                       //(step delay, M1, M2, M3, M4, M5, M6);
  braccio.ServoMovement(20,           0,  15, 180, 170, 0,  73);

  //Wait 1 second
  delay(1000);

  braccio.ServoMovement(20,           180,  165, 0, 0, 180,  10);

  //Wait 1 second
  delay(1000);
}
