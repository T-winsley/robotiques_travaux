#include <MeAuriga.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

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
unsigned long lastEncoder = 0;
unsigned long lastGyro = 0;
unsigned long lastLed = 0;

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

void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void setup()
{
  Serial.begin(115200);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);

  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);

  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);

  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);

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
      if (cT - stateStartTime < 1000)
      {
        avancerDroit(80);
      }
      else
      {
        stopMoteurs();
      }
      break;

    case PIVOT_1:
        // Pivoter de 90 degrés à droite
      break;

    case SEGMENT_2:
        // Avancer jusqu'à 30 cm de l'obstacle
      break;

    case PIVOT_2:
        // Pivoter de 90 degrés à gauche
      break;

    case SEGMENT_3:
      // Avancer jusqu'à la distance X
      break;

    case PIVOT_180:
      // Pivoter de 180 degrés
      break;

    case FIN:
      stopMoteurs();
      break;
  }
}


// Tâches
void encoderTask(unsigned long cT)
{
  // const unsigned long rate = 20;

  // if (cT - lastEncoder < rate)
  // {
  //   return;
  // }

  // lastEncoder = cT;

  Encoder_1.loop();
  Encoder_2.loop();

}


void gyroTask(unsigned long cT)
{
  const unsigned long rate = 20;

  if (cT - lastGyro < rate)
  {
    return;
  }

  lastGyro = cT;

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
  const unsigned long rate = 100;

  if (cT - lastLed < rate)
  {
    return;
  }

  lastLed = cT;


  
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

  Serial.print("Etat = ");
  Serial.println(currentState); 

  Serial.print("Encodeur 1 = ");
  Serial.println(Encoder_1.getCurPos());

  Serial.print("Encodeur 2 = ");
  Serial.println(Encoder_2.getCurPos());
}


// Mouvements
void avancerDroit(int pwm)
{
  Encoder_1.setMotorPwm(-pwm);
  Encoder_2.setMotorPwm(pwm);
}

bool pivoter(float angle)
{
  return false;
}


void stopMoteurs()
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}