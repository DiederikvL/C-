//================//
//joystick.cpp
//Diederik van Linden
//0970665
//TI2C
//================//


#include <cantino.h>
#include "joystick.h"

using namespace cantino;

Joystick::Joystick(int pX, int pY) {

  pinX = pX;
  pinY = pY;

  pinMode(pinX, INPUT);
  pinMode(pinY, INPUT);

}

int Joystick::readX() {
  
  x = map(analogRead(pinX), 0, 1023, -45, 50);
  return x;
}

int Joystick::readY() {
  
  y = map(analogRead(pinY), 0, 1023, 10, -10);
  return y;
}
