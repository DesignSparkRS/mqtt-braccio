/*
||
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @author         Alexander Brevig <abrevig@wiring.org.co>
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
|| @url            http://alexanderbrevig.com/
||
|| @description
|| |
|| | An interface for the TinkerKit Braccio robotic arm.
|| #
||
|| @license
|| |
|| | Copyright (c) 2016 - Alexander Brevig & Brett Hagman
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
|| #
||
|| @notes
|| |
|| | See Braccio.h for notes.
|| #
||
*/

#include "Braccio.h"
#include <Arduino.h>
#include <Servo.h>


Braccio::Braccio()
{
  servos[BASE] = NULL;
  servos[SHOULDER] = NULL;
  servos[ELBOW] = NULL;
  servos[WRIST] = NULL;
  servos[ROTATE] = NULL;
  servos[GRIPPER] = NULL;
  startTime = 0;
}


Braccio::Braccio(Servo &_base, Servo &_shoulder, Servo &_elbow, Servo &_wrist, Servo &_rotate, Servo &_gripper)
{
  servos[BASE] = &_base;
  servos[SHOULDER] = &_shoulder;
  servos[ELBOW] = &_elbow;
  servos[WRIST] = &_wrist;
  servos[ROTATE] = &_rotate;
  servos[GRIPPER] = &_gripper;
  startTime = 0;

  // Default position - all servos in middle
  for (int i = 0; i < SERVOCOUNT; i++)
  {
    currentPositions[i] = 90;
    targetPositions[i] = 90;
  }
}


void Braccio::setAll(int m1, int m2, int m3, int m4, int m5, int m6)
{
  targetPositions[BASE] = m1;
  targetPositions[SHOULDER] = m2;
  targetPositions[ELBOW] = m3;
  targetPositions[WRIST] = m4;
  targetPositions[ROTATE] = m5;
  targetPositions[GRIPPER] = m6;
}


void Braccio::home(btime_t timeSpentMoving, boolean blocking)
{
  // "safe" arm position (least moment with lowest center of mass)
  targetPositions[BASE] = 90;
  targetPositions[SHOULDER] = 45;
  targetPositions[ELBOW] = 180;
  targetPositions[WRIST] = 180;
  targetPositions[ROTATE] = 90;
  targetPositions[GRIPPER] = 45;
  moveTime = timeSpentMoving;

  if (blocking)
    while (update()); //make sure we get there
}


bool Braccio::update()
{
  // This is our processing state - i.e. do we still need to move the servos to their targets?
  bool processing = false;

  // let's just figure out our progress percentage based on the moveTime
  int progressInt = constrain(map(millis(), startTime, startTime + moveTime, 0, 1000), 0, 1000);
  float progress = (float)progressInt / 1000;

  for (int i = 0; i < SERVOCOUNT; i++)
  {
    // if the servo is defined
    if (servos[i])
    {
      // Move each servo to the new percentage
      uint16_t currentStep = progress * ((float)targetPositions[i] - currentPositions[i]);

      servos[i]->write(currentPositions[i] + currentStep);

      if (progress < 1)
      {
        processing = true;
      }
      else
      {
        currentPositions[i] = targetPositions[i];
      }
    }
  }
  return processing;
}


void Braccio::move(btime_t timeSpentMoving, boolean blocking)
{
  moveTime = timeSpentMoving == 0 ? 1 : timeSpentMoving;  // The difference can't be zero
  startTime = millis();
  if (blocking)
    while (update()); //go there now!
}


// Legacy support
int Braccio::ServoMovement(int dly, int Vbase, int Vshoulder, int Velbow, int Vwrist, int Vrotate, int Vgripper)
{
  int maxChange = 0;

  setAll(Vbase, Vshoulder, Velbow, Vwrist, Vrotate, Vgripper); // set all the targets

  // go through all of the differences, and see which is the greatest change
  for (int i = 0; i < SERVOCOUNT; i++)
  {
    int angleChange = targetPositions[i] - currentPositions[i];
    maxChange = max(maxChange, abs(angleChange));
  }

  if (maxChange > 0)
  {
    if (maxChange > 360) // just in case
      maxChange = 360;

    move((maxChange * dly)); // by default, a blocking call
  }

  return 1;
}

