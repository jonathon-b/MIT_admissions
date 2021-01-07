/*
Most relevant portion of this 
code is "moveTO," basic PID function.

"slewedDrive" function was a PID that 
limits aceleration as well (aka slew rate)

*/

class PidControl
{
public:
  int Error;
  int CurrentPosition;
  int Integral;
  float Derivative;
  int LastError;
  int slewOutput;
  int Output;
  int veloTarget;
  int veloTargetError;
  int innerArc;
  int outerArc;

  float Target;
  int veloMax;
  int veloStep;
  int PowerOut;
  float porportion;

  int slaveError;

  bool stopSignal = false;

  float theta;
  float kP, kI, kD;
  bool slewdisable;

  float ChassisRadius = 9.375;//inches

	//most basic form of PID
  void moveTo()
  {
    //positional portion
    Error = Target - CurrentPosition;
    
	if(abs(Error) < 10)
    {
	  //cap integral at 1000
      if(abs(Integral) < 1000)
      {
        Integral += Error;
      }
      else
      {
        Integral = sgn(Integral)*1000;
      }
    }
	
    Derivative = (Error - LastError)/100;
    LastError = Error;
    
	//motor voltage output
	Output = kP*Error + kI*Integral + kD*Derivative;
    
	//limit voltage to +-1000
	if(abs(Output) > 10000)
    {
      Output = sgn(Output)*10000;
    }
  }
  
  
  void pidStop()
  {
    stopSignal = true;
  }
 

  void arcCalc(int radius, int degrees)
  {
    theta = degrees*pi/180;
    innerArc = 360*(theta*(radius-ChassisRadius))/4*pi;
    outerArc = 360*(theta*(radius+ChassisRadius))/4*pi;
    porportion = innerArc/outerArc;

  }

  void pidInit(float P, float I, float D)
  {
    kP = P;
    kI = I;
    kD = D;
  }

  
  
  void slewedDrive()
  {
    //positional portion
    Error = Target - CurrentPosition;
    if(abs(Error) < 50)
    {
      if(abs(Integral) < 1000)
      {
        Integral += Error;
      }
      else
      {
        Integral = sgn(Integral)*1000;
      }
    }
    Derivative = (Error - LastError);
    LastError = Error;

    veloTarget = kP*Error + kI*Integral + kD*Derivative;
    if(abs(veloTarget) > veloMax)
    {
      veloTarget = sgn(veloTarget)*veloMax;
    }

    //velocity portion
    veloTargetError = veloTarget - Output;

    if(abs(veloTargetError) >= veloStep)
    {
      Output += sgn(veloTargetError)*veloStep;
    }
    else
    {
      Output += sgn(veloTargetError)*veloTargetError;
    }
    if(abs(Output) > abs(veloTarget))
    {
      PowerOut = veloTarget;
    }
    else
    {
      PowerOut = Output;
    }
  }

};