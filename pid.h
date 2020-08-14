//================//
//pid.h
//Diederik van Linden
//0970665
//TI2C
//================//

#include <cantino.h>

using namespace cantino;

class Pid {
  public:
    Pid ();
    int calculate (int ypr, int JSX);
    int JSX                 // value of Joystick
    float OldP = 0;         // Previous value used to calculate change in P //  DELTA P
    float P = 0;            //  Proportional component
    float I = 0;            //  Integral        just the sum of P over time
    float OldI = 0;         //  previous value of I for calculation of Delta I
    float D = 0;            //  Differential       D = P - OldP
    float bp = -810 ;       //   balance point
    int pid;

};
