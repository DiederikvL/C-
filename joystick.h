//================//
//joystick.h
//Diederik van Linden
//0970665
//TI2C
//================//

#include <cantino.h>

using namespace cantino;

class Joystick {
  public:
    Joystick (int pX, int pY);
    int readX();
    int readY();
  private:
    int pinX;
    int pinY;
    int x;
    int y;

};
