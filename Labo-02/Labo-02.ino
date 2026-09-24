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

enum State
{
  DRIVING,
  STOPPING,
  BACKING,
  TURNING
};

State currentState = DRIVING;

double distance = 0;

unsigned long currentTime = 0;
unsigned long lastSonar = 0;
unsigned long lastPrint = 0;
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

  Stop();
}


void loop()
{
  currentTime = millis();

  sonarTask(currentTime);
  serialTask(currentTime);
  stateManager(currentTime);
}


void stateManager(unsigned long cT)
{
  switch (currentState)
  {
    case DRIVING:

      if (distance > 0 && distance < 30)
      {
        RedLights();
        Stop();
        stateStartTime = cT;
        currentState = STOPPING;
      }

      else if (distance >= 30 && distance < 100)
      {
        YellowLights();
        Forward(vitesseLente);
      }

      else
      {
        GreenLights();
        Forward(vitesseCroisiere);
      }

      break;


    case STOPPING:

      RedLights();
      Stop();

      if (cT - stateStartTime >= 2000)
      {
        stateStartTime = cT;
        currentState = BACKING;
      }

      break;


    case BACKING:

      RedLights();
      Backward(vitesseLente);

      if (cT - stateStartTime >= 1000)
      {
        Stop();
        stateStartTime = cT;
        currentState = TURNING;
      }

      break;


    case TURNING:

      RedLights();
      TurnLeft(vitessePivot);

      if (cT - stateStartTime >= 1000)
      {
        Stop();
        currentState = DRIVING;
      }

      break;
  }
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


void GreenLights()
{
  led.setColor(0, 0, 0);

  for (int i = 6; i <= 11; i++)
  {
    led.setColorAt(i, 0, 255, 0);
  }

  led.show();
}


void YellowLights()
{
  led.setColor(0, 0, 0);

  for (int i = 0; i <= 5; i++)
  {
    led.setColorAt(i, 255, 255, 0);
  }

  led.show();
}


void RedLights()
{
  led.setColor(255, 0, 0);
  led.show();
}


void Forward(int speed)
{
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, speed);
}


void Backward(int speed)
{
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, speed);
}


void Stop()
{
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);
}


void TurnLeft(int speed)
{
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, speed);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, speed);
}