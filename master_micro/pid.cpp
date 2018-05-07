#ifndef _PID_SOURCE_
#define _PID_SOURCE_
#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;
class PIDImpl{
    public:
    PIDImpl( float dt, float max, float min, float Kp, float Kd, float Ki );
    ~PIDImpl();
    float calculate( float setpoint, float pv );
    private:
    float _dt;
    float _max;
    float _min;
    float _Kp;
    float _Kd;
    float _Ki;
    float _pre_error;
    float _integral;
};

PID::PID( float dt, float max, float min, float Kp, float Kd, float Ki ){
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
float PID::calculate( float setpoint, float pv ){
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
delete pimpl;
}
/**
 * Implementation
 */
PIDImpl::PIDImpl( float dt, float max, float min, float Kp, float Kd, float Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
    
}float PIDImpl::calculate( float setpoint, float pv ){
    // Calculate error
    float error = setpoint - pv;
    // Proportional term
    float Up = _Kp * error;
    // Integral term
    float Ui = _integral + error *_dt*_Ki;
    // Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Ud = _Kd * derivative;
    // Calculate total output
    float output = Up + Ui + Ud;

    // Save error to previous error
    _integral  = Ui;
    _pre_error = error;
    return output;
}

PIDImpl::~PIDImpl(){
}
#endif

