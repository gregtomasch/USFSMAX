/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef Alarms_h
#define Alarms_h

#include "def.h"
#include "Types.h"

#define LEDPIN_PINMODE                pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);                                             // Reverse logic for RGB LED; HIGH = OFF
#define LEDPIN_ON                     digitalWrite(LED_PIN, LOW);
#define LEDPIN_OFF                    digitalWrite(LED_PIN, HIGH);
#define LEDPIN_TOGGLE                 if(digitalRead(LED_PIN)) digitalWrite(LED_PIN, LOW); else digitalWrite(LED_PIN, HIGH);

class Alarms
{
  public:
                                      Alarms();
     static void                      blink_blueLED(uint8_t num, uint8_t ontime,uint8_t repeat);
     static void                      toggle_blueLED();
     static void                      blueLEDon();
     static void                      blueLEDoff();
  private:
};

#endif // Alarms_h
