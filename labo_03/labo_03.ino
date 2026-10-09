#include <MeAuriga.h>

enum State
{
  WAITING,
  SEGMENT_1,
  PIVOT_1,
  SEGMENT_2,
  PIVOT_2,
  SEGMENT_3,
  PIVOT_180,
  FIN
};

State currentState = WAITING;

//Variable
unsigned long currentTime = 0;
unsigned long stateStartTime = 0;


void setup()
{
  Serial.begin(115200);

  stateStartTime = millis();
}


void loop()
{
  currentTime = millis();

  encoderTask(currentTime);
  gyroTask(currentTime);
  sonarTask(currentTime);
  ledTask(currentTime);
  serialTask(currentTime);

  stateManager(currentTime);
}


void stateManager(unsigned long cT)
{
  switch (currentState)
  {
    case WAITING:
      break;

    case SEGMENT_1:
      break;

    case PIVOT_1:
      break;

    case SEGMENT_2:
      break;

    case PIVOT_2:
      break;

    case SEGMENT_3:
      break;

    case PIVOT_180:
      break;

    case FIN:
      break;
  }
}


// Tâches
void encoderTask(unsigned long cT)
{

}


void gyroTask(unsigned long cT)
{

}


void sonarTask(unsigned long cT)
{

}


void ledTask(unsigned long cT)
{

}


void serialTask(unsigned long cT)
{

}


// Mouvements
void avancerDroit(int pwm)
{

}


bool pivoter(float angle)
{
  return false;
}


void stopMoteurs()
{

}