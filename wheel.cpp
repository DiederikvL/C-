//================//
//wheel.cpp
//Diederik van Linden
//0970665
//TI2C
//================//

#include <cantino.h>
#include "wheel.h"
using namespace cantino;

Wheel::Wheel(int _pin1, int _pin2, int _pinpwm) {

  Pin1 = _pin1;   //pin motor to arduino  
  Pin2 = _pin2;
  Pinpwm = _pinpwm;

  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);
  pinMode(Pinpwm, OUTPUT);
}

void Wheel::setForward() {

  digitalWrite(Pin1, 1);
  digitalWrite(Pin2, 0);

}

void Wheel::setBackwards() {

  digitalWrite(Pin1, 0);
  digitalWrite(Pin2, 1);

}

void Wheel::stopWheel() {

  digitalWrite(Pinpwm, 0);

}

void Wheel::writePWM(int pwm) {

  analogWrite(Pinpwm, pwm);

}
