//================//
//wheel.h
//Diederik van Linden
//0970665
//TI2C
//================//

#include <cantino.h>

using namespace cantino;

class Wheel {
  public:
    Wheel (int _pin1, int _pin2, int _pinpwm);
    void writePWM(int pwm);
    void setForward();
    void setBackwards();
    void stopWheel();
  private:
    int Pin1;
    int Pin2;
    int Pinpwm;

};
