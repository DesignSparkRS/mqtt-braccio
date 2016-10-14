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
|| | There are limitations to some of the joints, as they are physically constrained.
|| | Specifically, the "shoulder" joint (M2) is restricted as it will strike the base at each
|| | limit.  The other joint that has limitations is the "gripper" (M6).  If any of the servos
|| | are put under load for any extended period of time, they will overheat and malfunction.
|| | Beware!
|| |
|| | Range:
|| | M6: 20 -> 95 (110 for good grip - but overloads servo)
|| | M5: 0 -> 180 (calibration required for centering)
|| | M4: 0 -> 180
|| | M3: 0 -> 180
|| | M2: 10 -> 170
|| | M1: 0 -> 180 (calibration needed)
|| |
|| | BH:
|| | Notes for making the library backward compatible with the initial release of the Braccio class:
|| | Because the original interface used a single delay amount between steps, it will
|| | be difficult to emulate it exactly with the API.
|| |
|| | original API:
|| | int ServoMovement(int delay, int Vbase,int Vshoulder, int Velbow, int Vwrist_ver, int Vwrist_rot, int Vgripper);
|| |
|| | Solution: go through the target positions, find the greatest angle change, multiply that angle
|| | change with "delay" -- use that as the timeSpentMoving
|| |
|| #
||
|| @todo
|| |
|| | - have offsets for "centering" the joints
|| |   e.g. setRotateCenter(83) -- this will translate a call for 90 degrees to 83 degrees
|| | - have a configurable default slew rate
|| | - make methods chainable
|| |
|| #
||
*/

#ifndef BRACCIO_H
#define BRACCIO_H

#include <Arduino.h>
#include <Servo.h>

class Braccio
{
  // Our time datatype
  typedef uint32_t btime_t;

  public:
    Braccio();
    Braccio(Servo &_base, Servo &_shoulder, Servo &_elbow, Servo &_wrist, Servo &_rotate, Servo &_gripper);

    void setAll(int m1, int m2, int m3, int m4, int m5, int m6);

    bool update();

    void home(btime_t timeSpentMoving = 1, boolean blocking = true);

    void move(btime_t timeSpentMoving = 1, boolean blocking = true);

    int baseNow() { return currentPositions[BASE]; }
    int base() { return targetPositions[BASE]; }
    void base(Servo &base) { servos[BASE] = &base; }
    void base(uint8_t target) { targetPositions[BASE] = target; }

    int shoulderNow() { return currentPositions[SHOULDER]; }
    int shoulder() { return targetPositions[SHOULDER]; }
    void shoulder(Servo &shoulder) { servos[SHOULDER] = &shoulder; }
    void shoulder(uint8_t target) { targetPositions[SHOULDER] = target; }

    int elbowNow() { return currentPositions[ELBOW]; }
    int elbow() { return targetPositions[ELBOW]; }
    void elbow(Servo &elbow) { servos[ELBOW] = &elbow; }
    void elbow(uint8_t target) { targetPositions[ELBOW] = target; }

    int wristNow() { return currentPositions[WRIST]; }
    int wrist() { return targetPositions[WRIST]; }
    void wrist(Servo &wrist) { servos[WRIST] = &wrist; }
    void wrist(uint8_t target) { targetPositions[WRIST] = target; }

    int rotateNow() { return currentPositions[ROTATE]; }
    int rotate() { return targetPositions[ROTATE]; }
    void rotate(Servo &rotate) { servos[ROTATE] = &rotate; }
    void rotate(uint8_t target) { targetPositions[ROTATE] = target; }

    int gripperNow() { return currentPositions[GRIPPER]; }
    int gripper() { return targetPositions[GRIPPER]; }
    void gripper(Servo &gripper) { servos[GRIPPER] = &gripper; }
    void gripper(uint8_t target) { targetPositions[GRIPPER] = target; }

    // legacy
    int ServoMovement(int dly, int Vbase, int Vshoulder, int Velbow, int Vwrist, int Vrotate, int Vgripper);

  private:
    static const uint8_t BASE = 0;
    static const uint8_t SHOULDER = 1;
    static const uint8_t ELBOW = 2;
    static const uint8_t WRIST = 3;
    static const uint8_t ROTATE = 4;
    static const uint8_t GRIPPER = 5;
    static const uint8_t SERVOCOUNT = 6;
    Servo *servos[6];
    uint16_t targetPositions[6];
    uint16_t currentPositions[6];
    btime_t startTime;
    btime_t moveTime;
};

#endif

