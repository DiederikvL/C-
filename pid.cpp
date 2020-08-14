//================//
//pid.cpp
//Diederik van Linden
//0970665
//TI2C
//================//

#include <cantino.h>
#include "pid.h"
using namespace cantino;

Pid::Pid(){

Serial.println("PID made");
  
}

int Pid::calculate(int ypr, int JSX) {

  OldP = P;                    // save value of P
  P = (ypr) - bp + JSX;  // update P from MPU add subtract target value (bp) to correct for balance (modified by STF). Joystick input gets added here aswel. 
  // dead band
  if ((P > -10) && (P < 10)) P = 0 ;

  OldI = I;                    // save old I
  I = I + (P * 0.05) ;
  I = I + ((I - OldI) * 2  );   // calulate new I
  if ( I >  250 ) I =  250;          // LIMIT  Stop I building up too high
  if ( I < -250 ) I = -250;          // or too low value
  D = P - OldP;                      //  D differential   change in P

  pid = ( P * 1.1 ) + ( I * 1  ) + ( D * 8 ) ; // P I D

  return pid;
}
