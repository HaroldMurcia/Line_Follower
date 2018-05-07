/* mbed Servo Library without using PWM pins
 * Copyright (c) 2010 Jasper Denkers
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef MBED_SERVO_H
#define MBED_SERVO_H

#include "mbed.h"

/** Class to control a servo on any pin, without using pwm
 *
 * Example:
 * @code
 * // Keep sweeping servo from left to right
 * #include "mbed.h"
 * #include "Servo.h"
 * 
 * Servo Servo1(p20);
 *
 * Servo1.Enable(1500,20000);
 *
 * while(1) {
 *     for (int pos = 1000; pos < 2000; pos += 25) {
 *         Servo1.SetPosition(pos);  
 *         wait_ms(20);
 *     }
 *     for (int pos = 2000; pos > 1000; pos -= 25) {
 *         Servo1.SetPosition(pos); 
 *         wait_ms(20); 
 *     }
 * }
 * @endcode
 */

class Servo {

public:
    /** Create a new Servo object on any mbed pin
     *
     * @param Pin Pin on mbed to connect servo to
     */
    Servo(PinName Pin);
    
    /** Change the position of the servo. Position in us
     *
     * @param NewPos The new value of the servos position (us)
     */
    void SetPosition(int NewPos);
    
    /** Enable the servo. Without enabling the servo won't be running. Startposition and period both in us.
     *
     * @param StartPos The position of the servo to start (us) 
     * @param Period The time between every pulse. 20000 us = 50 Hz(standard) (us)
     */
    void Enable(int StartPos, int Period);
    
    /** Disable the servo. After disabling the servo won't get any signal anymore
     *
     */
    void Disable();

private:
    void StartPulse();
    void EndPulse();

    int Position;
    DigitalOut ServoPin;
    Ticker Pulse;
    Timeout PulseStop;
};

#endif
