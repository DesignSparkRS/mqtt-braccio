/*
||
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
||
|| @description
|| |
|| | An example of controlling the Braccio programmed with the BraccioSerial example,
|| | controlled by this sketch running on an Arduino Esplora.
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
|| | Translate positions from joystick, linear pot, and two pairs of buttons into directional
|| | control for the Braccio.
|| |
|| | Simple packet control:
|| | Sj[-]nE
|| |
|| | S = Start of packet
|| | j = joint name (A,B,C,D,E,F)
|| | [-] = optional negative
|| | n = number of degrees (change) (integer)
|| | Z = End of packet
||
*/

#include <SoftwareSerial.h>
#include <Esplora.h>

#define PROCESSDELAY 15
#define XYANGLERATEMIN -2
#define XYANGLERATEMAX 2
#define SLIDERANGLEMIN 90
#define SLIDERANGLEMAX 180

// Set this to the pin you want to send data on (Pin 3 on the Esplora is on the "Out-A" connector)
SoftwareSerial mySerial = SoftwareSerial(255, 3); // RX, TX

int joyXOffset = 0;
int joyYOffset = 0;

int lastSliderValue = 0;


void setup()
{
  Serial.begin(9600);
  Serial.println(F("Esplora control for a Braccio using BraccioSerial example."));

  // Open serial communications
  mySerial.begin(9600);
}

void loop()
{
  int xValue = constrain(Esplora.readJoystickX() + joyXOffset, -512, 512);
  int yValue = constrain(Esplora.readJoystickY() + joyYOffset, -512, 512);
  int sliderValue = Esplora.readSlider();

  // If buttons 2 and 4 are pressed simultaneously, calibrate the X and Y joystick axes.
  if (!Esplora.readButton(SWITCH_2) && !Esplora.readButton(SWITCH_4))
  {
    joyXOffset = -Esplora.readJoystickX();
    joyYOffset = -Esplora.readJoystickY();
    Esplora.writeRGB(255, 0, 0);
    while (!Esplora.readButton(SWITCH_2) && !Esplora.readButton(SWITCH_4));  // wait until one button is released
    Esplora.writeRGB(0, 0, 0);
    return;  // start from the top
  }

  int sendX = -constrain(xValue / (500 / XYANGLERATEMAX), XYANGLERATEMIN, XYANGLERATEMAX);
  int sendY = -constrain(yValue / (500 / XYANGLERATEMAX), XYANGLERATEMIN, XYANGLERATEMAX);
  int sendSlider = map(sliderValue, 0, 1024, SLIDERANGLEMIN, SLIDERANGLEMAX);

  if (sendX != 0)
  {
    mySerial.print("SA");
    mySerial.print(sendX);
    mySerial.print('X');
    delay(PROCESSDELAY);
  }
  if (sendY != 0)
  {
    mySerial.print("SB");
    mySerial.print(sendY);
    mySerial.print('X');
    delay(PROCESSDELAY);
  }
  if (sendSlider != lastSliderValue)
  {
    lastSliderValue = sendSlider;
    mySerial.print("Sc");
    mySerial.print(sendSlider);
    mySerial.print('X');
    mySerial.print("Sd");
    mySerial.print(sendSlider);
    mySerial.print('X');
    delay(PROCESSDELAY);
  }
  if (!Esplora.readButton(SWITCH_2))
  {
    mySerial.print("SE2X");
    delay(PROCESSDELAY);
  }
  if (!Esplora.readButton(SWITCH_4))
  {
    mySerial.print("SE-2X");
    delay(PROCESSDELAY);
  }
  if (!Esplora.readButton(SWITCH_1))
  {
    mySerial.print("SF2X");
    delay(PROCESSDELAY);
  }
  if (!Esplora.readButton(SWITCH_3))
  {
    mySerial.print("SF-2X");
    delay(PROCESSDELAY);
  }
}

