#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);

const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;

const int vitesseCroisiere = 178;
const int vitesseLente = 89;
const int vitessePivot = 150;

double distance = 0;

unsigned long lastSonar = 0;
unsigned long lastPrint = 0;

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

unsigned long currentTime = 0;
unsigned long stateStartTime = 0;


void setup()
{
  Serial.begin(115200);

  led.setpin(LEDPIN);
  led.setColor(0, 0, 0);
  led.show();

  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in1, OUTPUT);
  pinMode(m2_in2, OUTPUT);

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

      stopMoteurs();

      if (cT - stateStartTime >= 3000)
      {
        stateStartTime = cT;
        currentState = SEGMENT_1;
      }

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
      stopMoteurs();
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
  const unsigned long rate = 100;

  if (cT - lastSonar < rate)
  {
    return;
  }

  lastSonar = cT;
  distance = sonar.distanceCm();
}

void ledTask(unsigned long cT)
{
  
}


void serialTask(unsigned long cT)
{
  const unsigned long rate = 250;

  if (cT - lastPrint < rate)
  {
    return;
  }

  lastPrint = cT;

  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
}


// Mouvements
void avancerDroit(int pwm){
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, pwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, pwm);

}


bool pivoter(float angle)
{
  return false;
}


void stopMoteurs()
{
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}