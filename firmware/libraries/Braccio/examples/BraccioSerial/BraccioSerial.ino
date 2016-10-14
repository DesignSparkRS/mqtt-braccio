/*
||
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
||
|| @description
|| |
|| | Basic serial message based control for Braccio arm.
|| #
||
|| @license
|| |
|| | Copyright (c) 2016 - Brett Hagman
|| |
|| | Permission is hereby granted, free of charge, to any person obtaining a copy of
|| | this software and associated documentation files (the "Software"), to deal in
|| | the Software without restriction, including without limitation the rights to
|| | use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
|| | the Software, and to permit persons to whom the Software is furnished to do so,
|| | subject to the following conditions:
|| |
|| | The above copyright notice and this permission notice shall be included in all
|| | copies or substantial portions of the Software.
|| |
|| | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
|| | IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
|| | FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
|| | COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
|| | IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
|| | CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
|| |
|| #
||
|| @notes
|| |
|| | The Plan (tm)
|| |
|| | Control can be relative or absolute positioning.
|| |
|| | Simple packet control:
|| | Sj[-]nX
|| |
|| | S = Start of packet
|| | j = joint name (A,B,C,D,E,F) - Lowercase = absolute, UPPERCASE = relative
|| | [-] = optional negative
|| | n = number of degrees (change) (integer)
|| | X = End of packet
|| |
|| | e.g. "SF-2X" - decrements the gripper angle by 2 degrees
|| |      "Sc45X" - sets the elbow joint to 45 degrees
|| |
|| | All data ignored until start of packet, packet is rejected if parameter constraints
|| | are not met.  Packet is complete after end of packet marker, then command is executed.
|| #
||
*/

#include <SoftwareSerial.h>
#include <Braccio.h>
#include <Servo.h>

// If you want to use a separate SoftwareSerial to control the arm.
SoftwareSerial mySerial = SoftwareSerial(15, 255); // RX, TX

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo rotate;
Servo gripper;

Braccio robot = Braccio(base, shoulder, elbow, wrist, rotate, gripper);

// These limits are for the two servos that have physical restrictions.
const int shoulderMinAngle = 10;
const int shoulderMaxAngle = 170;
const int gripperMinAngle = 20;
const int gripperMaxAngle = 95;

struct packet { char joint;
                int value; };


void setup()
{
  Serial.begin(9600);
  Serial.println(F("Braccio Serial Control"));
  mySerial.begin(9600);

  base.attach(11);
  shoulder.attach(10);
  elbow.attach(9);
  wrist.attach(6);
  rotate.attach(5);
  gripper.attach(3);

  Serial.print(F("Homing..."));
  robot.home(3000);
  Serial.println(F("Done"));
}


boolean getPacket(struct packet &outPacket)
{
  static int packetState = 0;
  static packet inPacket;

  boolean packetComplete = false;
  Stream *incoming = NULL;

  // Options for using the HardwareSerial, or a SoftwareSerial (defined above)
  if (Serial.available())
    incoming = &Serial;
  else if (mySerial.available())
    incoming = &mySerial;

  // We'll use a simple FSM (Finite State Machine) on the data coming in.
  // State - Description
  // 0 - Idle, waiting for start of packet 'S'
  // 1 - Waiting for joint name
  // 2 - Waiting for value
  // 3 - Waiting for end of packet 'X'

  if (incoming)
  {
    char c = incoming->peek();

    switch (packetState)
    {
      case 0:
        if (c == 'S')
          packetState = 1;
        incoming->read();
        break;
      case 1:
        if (c >= 'A' && c <= 'F')
        {
          inPacket.joint = c;
          inPacket.value = 1;
          packetState = 2;
        }
        else if (c >= 'a' && c <= 'f')
        {
          // absolute positioning
          inPacket.joint = c;
          packetState = 2;
        }
        else
        {
          // odd joint name, packet fails
          packetState = 0;
        }
        incoming->read();
        break;
      case 2:
        if (c == '-' || isdigit(c))
        {
          inPacket.value = incoming->parseInt();
          packetState = 3;
        }
        else
        {
          packetState = 0;
          incoming->read();
        }
        break;
      case 3:
        if (c == 'X')
        {
          packetComplete = true;
          // Copy incoming packet to output
          outPacket = inPacket;
        }
        packetState = 0;
        incoming->read();
        break;
      default:
        packetState = 0;
        incoming->read();
        break;
    }
  }
  return packetComplete;
}


void loop()
{
  packet myPacket;

  if (getPacket(myPacket))
  {
    Serial.print(F("Packet: "));
    Serial.print(myPacket.joint);
    Serial.print(' ');
    Serial.print(myPacket.value);
    parseCommand(myPacket);
  }
}


void parseCommand(packet &inPacket)
{
  int newValue = 0;

  switch (inPacket.joint)
  {
    case 'A':
      // Uppercase is relative to current position, so let's get the current value
      newValue = robot.base();
      // Because there is no 'break' here, we fall through to the next 'case'
    case 'a':
      // Lowercase is absolute positioning (newValue is set to 0 from the start)
      newValue += inPacket.value;
      newValue = constrain(newValue, 0, 180);
      robot.base(newValue);
      break;
    case 'B':
      newValue = robot.shoulder();
    case 'b':
      newValue += inPacket.value;
      newValue = constrain(newValue, shoulderMinAngle, shoulderMaxAngle);
      robot.shoulder(newValue);
      break;
    case 'C':
      newValue = robot.elbow();
    case 'c':
      newValue += inPacket.value;
      newValue = constrain(newValue, 0, 180);
      robot.elbow(newValue);
      break;
    case 'D':
      newValue = robot.wrist();
    case 'd':
      newValue += inPacket.value;
      newValue = constrain(newValue, 0, 180);
      robot.wrist(newValue);
      break;
    case 'E':
      newValue = robot.rotate();
    case 'e':
      newValue += inPacket.value;
      newValue = constrain(newValue, 0, 180);
      robot.rotate(newValue);
      break;
    case 'F':
      newValue = robot.gripper();
    case 'f':
      newValue += inPacket.value;
      newValue = constrain(newValue, gripperMinAngle, gripperMaxAngle);
      robot.gripper(newValue);
      break;
  }
  Serial.print(F(" Moving "));
  Serial.print(inPacket.joint);
  Serial.print(F(" to "));
  Serial.println(newValue);
  robot.move();
}

