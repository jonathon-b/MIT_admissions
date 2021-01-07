/*
Most relevant portion of this 
code is "moveTO," basic PID function.

"slewedDrive" function is a PID that 
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
	  
	  //limit I to 1000
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
	  
	  //calculate derivitive
    Derivative = (Error - LastError);
    LastError = Error;

    veloTarget = kP*Error + kI*Integral + kD*Derivative;
	  
	  //limit velocity
    if(abs(veloTarget) > veloMax)
    {
      veloTarget = sgn(veloTarget)*veloMax;
    }

    //velocity portion
    veloTargetError = veloTarget - Output;

	  //limit change in velocity (aceleration)
    if(abs(veloTargetError) >= veloStep)
    {
      Output += sgn(veloTargetError)*veloStep;
    }
    else
    {
      Output += sgn(veloTargetError)*veloTargetError;
    }
	  
	  /*
	  ensure veloMax is recognized; if veloTarget 
	  calculated by PID means robot acelerates too
	  fast, limit aceleration using the output of
	  slew rate control
	  */
	  
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
