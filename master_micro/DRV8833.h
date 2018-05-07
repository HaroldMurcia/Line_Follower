/* 
 * @file DRV8833.h
 * @author Oskar Lopez de Gamboa
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * mbed simple DRV8833 H-bridge motor controller
 * 
 *
 * PWM a un puente en H(DRV8833) conectado a los motores.
 * El comportamiento del driver es el siguiente:
 *  
 *          x_PWM1 x_PWM2    Mode
 *            0      0       Coast/Fast decay
 *            0      1       Reverse
 *            1      0       Forward
 *            1      1       Brake/slow decay
 *10/12/2013
*/
#ifndef MBED_DRV8833
#define MBED_DRV8833

#include "mbed.h"

#define COAST 1
#define BRAKE 0

/** Interface to control a standard DC motor 
 *  with an DRV8833 H-bridge motor controller
 *  using 2 PwmOuts 
 */
class DRV8833 {
public:

    /** Creates a DRV8833(H-bridge motor controller) control interface    
     *
     * @param pwm1 A PwmOut pin, tied to the AIN1 Logic input, controls state of AOUT1 
     * @param pwm2 A PwmOut pin, tied to the AIN2 Logic input controls state of AOUT2
     * 
     */
    DRV8833(PinName pwm1, PinName pwm2);
    
    /** Set the speed of the motor
     * 
     * @param speed The speed of the motor as a normalised value between -1.0 and 1.0
     */
    void speed(float speed);
    
    /** Set the period of the pwm duty cycle.
     *
     * Wrapper for PwmOut::period()
     *
     * @param seconds - Pwm duty cycle in seconds.
     */
    void period(float period);
    
    /** Brake the H-bridge coast or brake.
     * 
     * Defaults to coast.
     * @param mode - Braking mode.COAST(default)or BRAKE. 
     * 
     */
    void brake(int mode = COAST);

protected:
    PwmOut _pwm1;
    PwmOut _pwm2;
    
};

#endif

