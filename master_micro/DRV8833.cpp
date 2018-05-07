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


#include "DRV8833.h"

DRV8833::DRV8833(PinName pwm1, PinName pwm2):
        _pwm1(pwm1), _pwm2(pwm2) 
{

    // Set initial condition of PWM
    _pwm1.period(0.001);
    _pwm1 = 0;
    _pwm2.period(0.001);
    _pwm2 = 0;
     
}

void DRV8833::speed(float speed) 
{
    if (speed > 0.0)
    {
        _pwm1 = 0;
        _pwm2 = abs(speed);
    }
    else
    {
      _pwm1 = abs(speed);
      _pwm2 = 0;
    }
}
void DRV8833::period(float period){

    _pwm1.period(period);
    _pwm2.period(period);
}

void DRV8833::brake(int mode){

    if(mode == COAST)
    {
        _pwm1 = 0;
        _pwm2 = 0;     
    }
    else if(mode == BRAKE)
    {
        _pwm1 = 1;
        _pwm2 = 1; 
    }

}

