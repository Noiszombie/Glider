#include <Servo.h>

#define M1_STEP_PIN 13
#define M1_ENABLE_PIN 10
#define M1_DIR_PIN 12
#define CALIBRATION_TURN_MIN_ANGLE 1

using namespace std;

// Static variable


// Engine class (for three engines, class methods get angles in degrees and full speed divider)
class engine
{
  public:
    // Constructor
    engine(int stepP, int enableP, int dirP, int frontP = -1, int backP = -1, int frontPM = -1, int backPM = -1)
      : StepPin (stepP), EnablePin (enableP), DirectionPin(dirP), FrontPin(frontP), BackPin(backP), FrontPinMode(frontPM), BackPinMode(backPM)
    {
      // cout « "Engine addition" « endl;
      angle = 0;
      speedDivider = 0;
      turn = 0;
      switcher = 0;
      TurnMax = 0;
      TurnMin = 0;
      sD = 0;
    }

    void installization()
    {
      pinMode(EnablePin, OUTPUT); //Enable
      pinMode(StepPin, OUTPUT); //Step
      pinMode(DirectionPin, OUTPUT); //Direction
      pinMode(FrontPin, INPUT);
      pinMode(BackPin, INPUT);
      digitalWrite(EnablePin, LOW); //Turn on voltage
    }

//    void Calibration(int MinAngle, int speedDiv)
//    {
//      if (FrontPin == - 1 && BackPin == -1)
//      {
//        return;
//      }
//      if (FrontPin != -1)
//      {
//        while(DigitalRead(FrontPin) == FrontPinMode)
//        {
//          movee(MinAngle,speedDiv);
//          while(angle > ){}
//          TurnMax += MinAngle;
//        } 
//      }
//    }

    // Return speed divider
    unsigned int getSpeedE()
    {
      return speedDivider;
    }

    // Return angle
    double getAngle()
    {
      return angle;
    }

    // Do one step in one side(+1.8 / 4 degrees)
    void dicAngle()
    {
      angle -= 1;
      turn += 1;
    }

    // Do one step in other side(-1.8 degrees / 2 degrees)
    void incAngle()
    {
      angle += 1;
      turn -= 1;
    }

    //Emergency stop
    void stop ()
    {
      angle = 0;
      speedDivider = 0;
      sD = 0;
    }
    
    // Set angle and speed of moving and start
    void movee(double ang, unsigned int sd)
    {
      angle = ang;
      speedDivider = sd;
    }

    void engineStepPos()
    {
      switcher = !switcher;
      digitalWrite(DirectionPin, HIGH);
      digitalWrite(StepPin, switcher); // First - low, following - high - /is it normal?/ In first interrupt sD = 1, and it's normal. When we have even number, last bit in sD ever is 0
      this->dicAngle();
    }

    void engineStepNeg()
    {
      switcher = !switcher;
      digitalWrite(DirectionPin, LOW);
      digitalWrite(StepPin, switcher); // First - low, following - high - /is it normal?/ In first interrupt sD = 1, and it's normal. When we have even number, last bit in sD ever is 0
      this->incAngle();
    }
    
    void incSD()
    {
      sD++;
    }

    void getSD()
    {
      return sD;
    }

    // Reset speedDivider
    void resetSD()
    {
      speedDivider = 0;
      sD = 0; 
    }
  protected:
    int turn;
    int angle;
    unsigned int speedDivider;
    bool switcher;
    int TurnMax;
    int TurnMin;
    //Pins installization
    int StepPin;
    int EnablePin;
    int DirectionPin;
    int FrontPin;
    int BackPin;
    int FrontPinMode;
    int BackPinMode;
    unsigned int sD ;
};


// Object installization. Set number of pins (StepPin, EnablePin, DirectionPin)
engine piston(13, 10, 12);
engine fin();
engine mass();


void setup()
{
  Serial.begin(9600);
  engine piston(13, 10, 12);
  engine fin();
  engine mass();
  piston.installization();
  OCR0A = 0xAF; // If we want max speed, we can reduce the timer time by 2 times, because we increased the number of interrupts by 2 times
  TIMSK0 |= _BV(OCIE0A);
}


SIGNAL(TIMER0_COMPA_vect)
{
  if (piston.getAngle() != 0)
  {
    piston.incSD; //Counter for speed divider
  }

  // In one side
  if ((piston.getAngle() != 0) && (piston.getAngle() > 0) && (piston.getSD % piston.getSpeedE() == 0))
  {
    piston.engineStepPos(); // Move in positive direction
  }

  // In other side
  if ((piston.getAngle() != 0) && (piston.getAngle() < 0) && (piston.getSD % piston.getSpeedE() == 0))
  {
    piston.engineStepNeg(); // Move in negative direction
  }

  if (piston.getSD == piston.getAngle()*piston.getSpeedE() * 2)
  {
    piston.resetSD();
  }
}

void loop()
{
  piston.movee(1000, 2);
  delay(5000);
  piston.movee(-1000, 1);
  delay(5000);
}
