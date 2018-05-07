/* m3pi Library
 *
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

#include "mbed.h"
#include "m3pi.h"

m3pi::m3pi(PinName nrst, PinName tx, PinName rx) :  Stream("m3pi"), _nrst(nrst), _ser(tx, rx)  {
    _ser.baud(115200);
    reset();
}

m3pi::m3pi() :  Stream("m3pi"), _nrst(p23), _ser(p9, p10)  {
    _ser.baud(115200);
    reset();
}


void m3pi::reset () {
    _nrst = 0;
    wait (0.01);
    _nrst = 1;
    wait (0.1);
}

void m3pi::left_motor (float speed) {
    motor(0,speed);
}

void m3pi::right_motor (float speed) {
    motor(1,speed);
}

void m3pi::forward (float speed) {
    motor(0,speed);
    motor(1,speed);
}

void m3pi::backward (float speed) {
    motor(0,-1.0*speed);
    motor(1,-1.0*speed);
}

void m3pi::left (float speed) {
    motor(0,speed);
    motor(1,-1.0*speed);
}

void m3pi::right (float speed) {
    motor(0,-1.0*speed);
    motor(1,speed);
}

void m3pi::stop (void) {
    motor(0,0.0);
    motor(1,0.0);
}

void m3pi::motor (int motor, float speed) {
    char opcode = 0x0;
    if (speed > 0.0) {
        if (motor==1)
            opcode = M1_FORWARD;
        else
            opcode = M2_FORWARD;
    } else {
        if (motor==1)
            opcode = M1_BACKWARD;
        else
            opcode = M2_BACKWARD;
    }
    unsigned char arg = 0x7f * abs(speed);

    _ser.putc(opcode);
    _ser.putc(arg);
}

float m3pi::battery() {
    _ser.putc(SEND_BATTERY_MILLIVOLTS);
    char lowbyte = _ser.getc();
    char hibyte  = _ser.getc();
    float v = ((lowbyte + (hibyte << 8))/1000.0);
    return(v);
}

float m3pi::line_position() {
    int pos = 0;
    _ser.putc(SEND_LINE_POSITION);
    pos = _ser.getc();
    pos += _ser.getc() << 8;
    
    float fpos = ((float)pos);
    return(fpos);
}

char m3pi::sensor_auto_calibrate() {
    _ser.putc(AUTO_CALIBRATE);
    return(_ser.getc());
}


void m3pi::calibrate(void) {
    _ser.putc(PI_CALIBRATE);
}

void m3pi::reset_calibration() {
    _ser.putc(LINE_SENSORS_RESET_CALIBRATION);
}

void m3pi::PID_start(int max_speed, int a, int b, int c, int d) {
    _ser.putc(max_speed);
    _ser.putc(a);
    _ser.putc(b);
    _ser.putc(c);
    _ser.putc(d);
}

void m3pi::PID_stop() {
    _ser.putc(STOP_PID);
}

float m3pi::pot_voltage(void) {
    int volt = 0;
    _ser.putc(SEND_TRIMPOT);
    volt = _ser.getc();
    volt += _ser.getc() << 8;
    return(volt);
}


void m3pi::leds(int val) {

    BusOut _leds(p20,p19,p18,p17,p16,p15,p14,p13);
    _leds = val;
}


void m3pi::locate(int x, int y) {
    _ser.putc(DO_LCD_GOTO_XY);
    _ser.putc(x);
    _ser.putc(y);
}

void m3pi::cls(void) {
    _ser.putc(DO_CLEAR);
}

int m3pi::print (char* text, int length) {
    _ser.putc(DO_PRINT);  
    _ser.putc(length);       
    for (int i = 0 ; i < length ; i++) {
        _ser.putc(text[i]); 
    }
    return(0);
}

int m3pi::_putc (int c) {
    _ser.putc(DO_PRINT);  
    _ser.putc(0x1);       
    _ser.putc(c);         
    wait (0.001);
    return(c);
}

int m3pi::_getc (void) {
    char r = 0;
    return(r);
}

int m3pi::putc (int c) {
    return(_ser.putc(c));
}

int m3pi::getc (void) {
    return(_ser.getc());
}

void m3pi::Read_Sensor(int *sensor) {
    
    _ser.putc(SEND_RAW_SENSOR_VALUES); 
    sensor[0] = _ser.getc();
    sensor[0] += _ser.getc() << 8;
    sensor[1] = _ser.getc();
    sensor[1] += _ser.getc() << 8;
    sensor[2] = _ser.getc();
    sensor[2] += _ser.getc() << 8;
    sensor[3] = _ser.getc();
    sensor[3] += _ser.getc() << 8;
    sensor[4] = _ser.getc();
    sensor[4] += _ser.getc() << 8;
    sensor[5] = _ser.getc();
    sensor[5] += _ser.getc() << 8;
    sensor[6] = _ser.getc();
    sensor[6] += _ser.getc() << 8;
    sensor[7] = _ser.getc();
    sensor[7] += _ser.getc() << 8;
    return;
}

void m3pi::Max_Values_Calibrate(int *sensor_max) {
    
    _ser.putc(MAX_VALUES_CALIBRATE); 
    sensor_max[0] = _ser.getc();
    sensor_max[0] += _ser.getc() << 8;
    sensor_max[1] = _ser.getc();
    sensor_max[1] += _ser.getc() << 8;
    sensor_max[2] = _ser.getc();
    sensor_max[2] += _ser.getc() << 8;
    sensor_max[3] = _ser.getc();
    sensor_max[3] += _ser.getc() << 8;
    sensor_max[4] = _ser.getc();
    sensor_max[4] += _ser.getc() << 8;
    sensor_max[5] = _ser.getc();
    sensor_max[5] += _ser.getc() << 8;
    sensor_max[6] = _ser.getc();
    sensor_max[6] += _ser.getc() << 8;
    sensor_max[7] = _ser.getc();
    sensor_max[7] += _ser.getc() << 8;
    return;
}

void m3pi::Min_Values_Calibrate(int *sensor_min) {
    
    _ser.putc(MIN_VALUES_CALIBRATE); 
    sensor_min[0] = _ser.getc();
    sensor_min[0] += _ser.getc() << 8;
    sensor_min[1] = _ser.getc();
    sensor_min[1] += _ser.getc() << 8;
    sensor_min[2] = _ser.getc();
    sensor_min[2] += _ser.getc() << 8;
    sensor_min[3] = _ser.getc();
    sensor_min[3] += _ser.getc() << 8;
    sensor_min[4] = _ser.getc();
    sensor_min[4] += _ser.getc() << 8;
    sensor_min[5] = _ser.getc();
    sensor_min[5] += _ser.getc() << 8;
    sensor_min[6] = _ser.getc();
    sensor_min[6] += _ser.getc() << 8;
    sensor_min[7] = _ser.getc();
    sensor_min[7] += _ser.getc() << 8;
    return;
}






#ifdef MBED_RPC
const rpc_method *m3pi::get_rpc_methods() {
    static const rpc_method rpc_methods[] = {{ "forward", rpc_method_caller<m3pi, float, &m3pi::forward> },
        { "backward", rpc_method_caller<m3pi, float, &m3pi::backward> },
        { "left", rpc_method_caller<m3pi, float, &m3pi::left> },
        { "right", rpc_method_caller<m3pi, float, &m3pi::right> },
        { "stop", rpc_method_caller<m3pi, &m3pi::stop> },
        { "left_motor", rpc_method_caller<m3pi, float, &m3pi::left_motor> },
        { "right_motor", rpc_method_caller<m3pi, float, &m3pi::right_motor> },
        { "battery", rpc_method_caller<float, m3pi, &m3pi::battery> },
        { "line_position", rpc_method_caller<float, m3pi, &m3pi::line_position> },
        { "sensor_auto_calibrate", rpc_method_caller<char, m3pi, &m3pi::sensor_auto_calibrate> },


        RPC_METHOD_SUPER(Base)
    };
    return rpc_methods;
}
#endif

