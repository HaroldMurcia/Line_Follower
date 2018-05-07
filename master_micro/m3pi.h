/* mbed m3pi Library
 * Copyright (c) 2007-2010 cstyles
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

#ifndef M3PI_H
#define M3PI_H

#include "mbed.h"
#include "platform.h"

#ifdef MBED_RPC
#include "rpc.h"
#endif

#define SEND_SIGNATURE 0x81
#define SEND_RAW_SENSOR_VALUES 0x86
#define SEND_TRIMPOT 0xB0
#define SEND_BATTERY_MILLIVOLTS 0xB1
#define DO_PLAY 0xB3
#define PI_CALIBRATE 0xB4
#define DO_CLEAR 0xB7
#define DO_PRINT 0xB8
#define DO_LCD_GOTO_XY 0xB9
#define LINE_SENSORS_RESET_CALIBRATION 0xB5
#define SEND_LINE_POSITION 0xB6
#define AUTO_CALIBRATE 0xBA
#define SET_PID 0xBB
#define STOP_PID 0xBC
#define M1_FORWARD 0xC1
#define M1_BACKWARD 0xC2
#define M2_FORWARD 0xC5
#define M2_BACKWARD 0xC6

#define MAX_VALUES_CALIBRATE 0xB2
#define MIN_VALUES_CALIBRATE 0xB3



/** m3pi control class
 *
 * Example:
 * @code
 * // Drive the m3pi forward, turn left, back, turn right, at half speed for half a second

   #include "mbed.h"
   #include "m3pi.h"

   m3pi pi;

   int main() {

     wait(0.5);

     pi.forward(0.5);
     wait (0.5);
     pi.left(0.5);
     wait (0.5);
     pi.backward(0.5);
     wait (0.5);
     pi.right(0.5);
     wait (0.5);

     pi.stop();

 }
 * @endcode
 */
class m3pi :  public Stream {

    // Public functions
public:

    /** Create the m3pi object connected to the default pins
     *
     * @param nrst GPIO pin used for reset. Default is p23
     * @param tx Serial transmit pin. Default is p9
     * @param rx Serial receive pin. Default is p10
     */
    m3pi();


    /** Create the m3pi object connected to specific pins
     *
     */
    m3pi(PinName nrst, PinName tx, PinName rx);



    /** Force a hardware reset of the 3pi
     */
    void reset (void);

    /** Directly control the speed and direction of the left motor
     *
     * @param speed A normalised number -1.0 - 1.0 represents the full range.
     */
    void left_motor (float speed);

    /** Directly control the speed and direction of the right motor
     *
     * @param speed A normalised number -1.0 - 1.0 represents the full range.
     */
    void right_motor (float speed);

    /** Drive both motors forward as the same speed
     *
     * @param speed A normalised number 0 - 1.0 represents the full range.
     */
    void forward (float speed);

    /** Drive both motors backward as the same speed
     *
     * @param speed A normalised number 0 - 1.0 represents the full range.
     */
    void backward (float speed);

    /** Drive left motor backwards and right motor forwards at the same speed to turn on the spot
     *
     * @param speed A normalised number 0 - 1.0 represents the full range.
     */
    void left (float speed);

    /** Drive left motor forward and right motor backwards at the same speed to turn on the spot
     * @param speed A normalised number 0 - 1.0 represents the full range.
     */
    void right (float speed);

    /** Stop both motors
     *
     */
    void stop (void);

    /** Read the voltage of the potentiometer on the 3pi
     * @returns voltage as a float
     *
     */
    float pot_voltage(void);

    /** Read the battery voltage on the 3pi
     * @returns battery voltage as a float
     */
    float battery(void);

    /** Read the position of the detected line
     * @returns position as A normalised number -1.0 - 1.0 represents the full range.
     *  -1.0 means line is on the left, or the line has been lost
     *   0.0 means the line is in the middle
     *   1.0 means the line is on the right
     */
    float line_position (void);


    /** Calibrate the sensors. This turns the robot left then right, looking for a line
     *
     */
    char sensor_auto_calibrate (void);

    /** Set calibration manually to the current settings.
     *
     */
    void calibrate(void);

    /** Clear the current calibration settings
     *
     */
    void reset_calibration (void);

    void PID_start(int max_speed, int a, int b, int c, int d);

    void PID_stop();

    /** Write to the 8 LEDs
     *
     * @param leds An 8 bit value to put on the LEDs
     */
    void leds(int val);

    /** Locate the cursor on the 8x2 LCD
     *
     * @param x The horizontal position, from 0 to 7
     * @param y The vertical position, from 0 to 1
     */
    void locate(int x, int y);

    /** Clear the LCD
     *
     */
    void cls(void);

    /** Send a character directly to the 3pi serial interface
     * @param c The character to send to the 3pi
     */
    int putc(int c);

    /** Receive a character directly to the 3pi serial interface
     * @returns c The character received from the 3pi
     */
    int getc();

    /** Send a string buffer to the 3pi serial interface
     * @param text A pointer to a char array
     * @param int The character to send to the 3pi
     */
    int print(char* text, int length);
    
    /**
     *
     *
     */
     void Read_Sensor (int *sensor);
     
     /**
      *
      *
      *
      */
     void Max_Values_Calibrate (int *sensor_max);
     
     /**
      *
      *
      *
      */
     void Min_Values_Calibrate (int *sensor_min);

#ifdef MBED_RPC
    virtual const struct rpc_method *get_rpc_methods();
#endif

private :

    DigitalOut _nrst;
    Serial _ser;
    
    void motor (int motor, float speed);
    virtual int _putc(int c);
    virtual int _getc();

};

#endif
