#include "Servo.h"
#include "mbed.h"

    Servo::Servo(PinName Pin) : ServoPin(Pin) {}

    void Servo::SetPosition(int Pos) {
        Position = Pos;
    }

    void Servo::StartPulse() {
        ServoPin = 1;
        PulseStop.attach_us(this, &Servo::EndPulse, Position);
    }

    void Servo::EndPulse() {
        ServoPin = 0;
    }

    void Servo::Enable(int StartPos, int Period) {
        Position = StartPos;
        Pulse.attach_us(this, &Servo::StartPulse, Period);
    }

    void Servo::Disable() {
        Pulse.detach();
    }
