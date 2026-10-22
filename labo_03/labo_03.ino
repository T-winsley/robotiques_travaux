
#include <MeAuriga.h>
#include <Wire.h>

#define LEDNUM 12
#define LEDPIN 44

// laboratoire 03 - livraison autonome
// da : 2421657
// deux derniers chiffres : 57
// da[6] = 7 (impair)
// premier pivot : droite 90 degres
// deuxieme pivot : gauche 90 degres
// dernier pivot : 180 degres

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeGyro gyro(0, 0x69);

// constantes

const float X = 120.0;
const float diametreRoue = 6.5;
const float distanceObstacle = 30.0;

// meme vitesse pour les trois segments
const int vitesse = 100;

// vitesse utilisee seulement pour pivoter
const int vitessePivot = 50;

// machine a etats

enum Etat
{
  ATTENTE,
  SEGMENT_1,
  PIVOT_DROITE,
  SEGMENT_2,
  PIVOT_GAUCHE,
  SEGMENT_3,
  PIVOT_180,
  FIN,
  ERREUR
};

Etat etat = ATTENTE;

unsigned long debutEtat = 0;
unsigned long debutPivot = 0;

long departGauche = 0;
long departDroite = 0;

bool premiereFois = true;

float angleDepart = 0;
float distanceDebutSegment2 = 0;
float distanceSonar = 0;

bool segment1Termine = false;
bool segment2Termine = false;
bool segment3Termine = false;

// interruptions des encodeurs

void interruptionDroite()
{
  if (digitalRead(encoderRight.getPortB()) == 0)
    encoderRight.pulsePosMinus();
  else
    encoderRight.pulsePosPlus();
}

void interruptionGauche()
{
  if (digitalRead(encoderLeft.getPortB()) == 0)
    encoderLeft.pulsePosMinus();
  else
    encoderLeft.pulsePosPlus();
}

// configuration des encodeurs

void configEncodeurs()
{
  attachInterrupt(
    encoderRight.getIntNum(),
    interruptionDroite,
    RISING
  );

  attachInterrupt(
    encoderLeft.getIntNum(),
    interruptionGauche,
    RISING
  );

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  encoderLeft.setPulse(9);
  encoderRight.setPulse(9);

  encoderLeft.setRatio(39.267);
  encoderRight.setRatio(39.267);

  encoderLeft.setPosPid(1.8, 0, 1.2);
  encoderRight.setPosPid(1.8, 0, 1.2);

  encoderLeft.setSpeedPid(0.18, 0, 0);
  encoderRight.setSpeedPid(0.18, 0, 0);
}

// arret des moteurs

void Stop()
{
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

// avancer droit avec le gyroscope

void goStraight(short speed = 100, short firstRun = 0)
{
  static double localGoal = 0.0;
  static double previousError = 0.0;

  const double kp = 10.0;
  const double kd = 1.0;

  if (firstRun)
  {
    localGoal = gyro.getAngleZ();
    previousError = 0;
    return;
  }

  double error = gyro.getAngleZ() - localGoal;
  double output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setMotorPwm(
    constrain(speed - output, -255, 255)
  );

  encoderRight.setMotorPwm(
    constrain(-speed - output, -255, 255)
  );
}

// distance avec les encodeurs

float distanceParcourue()
{
  float gauche =
    abs(encoderLeft.getCurPos() - departGauche);

  float droite =
    abs(encoderRight.getCurPos() - departDroite);

  float moyenne = (gauche + droite) / 2.0;

  return moyenne * PI * diametreRoue / 360.0;
}

void initialiserDistance()
{
  departGauche = encoderLeft.getCurPos();
  departDroite = encoderRight.getCurPos();
}

// changement d'etat

void changerEtat(Etat nouvelEtat, unsigned long cT)
{
  Stop();

  etat = nouvelEtat;
  debutEtat = cT;
  premiereFois = true;
}

// attente de 3 secondes

void attenteTask(unsigned long cT)
{
  Stop();

  if (cT - debutEtat >= 3000)
  {
    changerEtat(SEGMENT_1, cT);
  }
}

// segment 1 - 120 cm

void segment1Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    initialiserDistance();
    goStraight(vitesse, 1);

    premiereFois = false;
    lastTime = cT;
    return;
  }

  if (cT - lastTime < 20)
    return;

  lastTime = cT;

  if (distanceParcourue() >= X)
  {
    segment1Termine = true;
    changerEtat(PIVOT_DROITE, cT);
    return;
  }

  goStraight(vitesse);
}

// fonction commune aux pivots

bool effectuerPivot(unsigned long cT, int direction, float angleCible)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    Stop();

    angleDepart = gyro.getAngleZ();
    debutPivot = cT;

    premiereFois = false;
    lastTime = cT;
    return false;
  }

  if (cT - lastTime < 20)
    return false;

  lastTime = cT;

  float angleTourne =
    abs(gyro.getAngleZ() - angleDepart);

  // arret lorsque l'angle demande est atteint
  if (angleTourne >= angleCible)
  {
    Stop();
    return true;
  }

  // securite contre une rotation sans fin
  if (cT - debutPivot >= 5000)
  {
    changerEtat(ERREUR, cT);
    return false;
  }

  // direction = 1 : droite
  // direction = -1 : gauche
  encoderLeft.setMotorPwm(
    direction * vitessePivot
  );

  encoderRight.setMotorPwm(
    direction * vitessePivot
  );

  return false;
}

// premier pivot - droite 90 degres

void pivotDroiteTask(unsigned long cT)
{
  if (effectuerPivot(cT, 1, 90.0))
  {
    changerEtat(SEGMENT_2, cT);
  }
}

// segment 2 - arret a 30 cm

void segment2Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    Stop();
    initialiserDistance();

    distanceDebutSegment2 = sonar.distanceCm();
    distanceSonar = distanceDebutSegment2;

    if (distanceSonar <= 0)
    {
      changerEtat(ERREUR, cT);
      return;
    }

    if (distanceSonar <= distanceObstacle)
    {
      segment2Termine = true;
      changerEtat(PIVOT_GAUCHE, cT);
      return;
    }

    goStraight(vitesse, 1);

    premiereFois = false;
    lastTime = cT;
    return;
  }

  if (cT - lastTime < 50)
    return;

  lastTime = cT;

  distanceSonar = sonar.distanceCm();

  // arret a 30 cm de l'obstacle
  if (distanceSonar > 0 &&
      distanceSonar <= distanceObstacle)
  {
    segment2Termine = true;
    changerEtat(PIVOT_GAUCHE, cT);
    return;
  }

  // mesure invalide : arret de securite
  if (distanceSonar <= 0)
  {
    changerEtat(ERREUR, cT);
    return;
  }

  // meme vitesse que les segments 1 et 3
  goStraight(vitesse);
}

// deuxieme pivot - gauche 90 degres

void pivotGaucheTask(unsigned long cT)
{
  if (effectuerPivot(cT, -1, 90.0))
  {
    changerEtat(SEGMENT_3, cT);
  }
}

// segment 3 - 120 cm

void segment3Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    initialiserDistance();
    goStraight(vitesse, 1);

    premiereFois = false;
    lastTime = cT;
    return;
  }

  if (cT - lastTime < 20)
    return;

  lastTime = cT;

  if (distanceParcourue() >= X)
  {
    segment3Termine = true;
    changerEtat(PIVOT_180, cT);
    return;
  }

  goStraight(vitesse);
}

// dernier pivot - 180 degres

void pivot180Task(unsigned long cT)
{
  if (effectuerPivot(cT, 1, 180.0))
  {
    changerEtat(FIN, cT);
  }
}

// fin et erreur

void finTask(unsigned long cT)
{
  Stop();
}

void erreurTask(unsigned long cT)
{
  Stop();
}

// machine a etats (fsm)

void stateTask(unsigned long cT)
{
  switch (etat)
  {
    case ATTENTE:
      attenteTask(cT);
      break;

    case SEGMENT_1:
      segment1Task(cT);
      break;

    case PIVOT_DROITE:
      pivotDroiteTask(cT);
      break;

    case SEGMENT_2:
      segment2Task(cT);
      break;

    case PIVOT_GAUCHE:
      pivotGaucheTask(cT);
      break;

    case SEGMENT_3:
      segment3Task(cT);
      break;

    case PIVOT_180:
      pivot180Task(cT);
      break;

    case FIN:
      finTask(cT);
      break;

    case ERREUR:
      erreurTask(cT);
      break;
  }
}

// gyroscope

void gyroTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 20)
    return;

  lastTime = cT;
  gyro.update();
}

// del de progression

void ledTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 50)
    return;

  lastTime = cT;

  led.setColor(0, 0, 0, 0);

  // demarrage : toutes les del jaunes
  if (etat == ATTENTE)
  {
    led.setColor(0, 20, 20, 0);
  }

  // barre de progression segment 1
  if (etat == SEGMENT_1)
  {
    float progression =
      constrain(distanceParcourue() / X, 0.0, 1.0);

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
      led.setColor(i, 0, 255, 0);
  }

  // barre de progression segment 2
  if (etat == SEGMENT_2)
  {
    float distancePrevue =
      distanceDebutSegment2 - distanceObstacle;

    if (distancePrevue > 0)
    {
      float progression = constrain(
        distanceParcourue() / distancePrevue,
        0.0,
        1.0
      );

      int nombreLeds = progression * 7;

      for (int i = 1; i <= nombreLeds; i++)
        led.setColor(i, 0, 255, 0);
    }
  }

  // barre de progression segment 3
  if (etat == SEGMENT_3)
  {
    float progression =
      constrain(distanceParcourue() / X, 0.0, 1.0);

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
      led.setColor(i, 0, 255, 0);
  }

  // del des segments termines
  if (segment1Termine)
    led.setColor(12, 0, 255, 0);

  if (segment2Termine)
    led.setColor(11, 0, 255, 0);

  if (segment3Termine)
    led.setColor(10, 0, 255, 0);

  // fin : clignotement a 10 hz
  if (etat == FIN)
  {
    bool allume = (cT / 50) % 2 == 0;

    for (int i = 8; i <= 12; i++)
    {
      if (allume)
        led.setColor(i, 0, 255, 0);
      else
        led.setColor(i, 0, 0, 0);
    }
  }

  // erreur : del rouges
  if (etat == ERREUR)
  {
    led.setColor(0, 255, 0, 0);
  }

  led.show();
}

// affichage dans le moniteur serie

void serialTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 250)
    return;

  lastTime = cT;

  Serial.print("Etat : ");
  Serial.print(etat);

  Serial.print(" | Distance : ");
  Serial.print(distanceParcourue());

  Serial.print(" cm | Angle Z : ");
  Serial.print(gyro.getAngleZ());

  Serial.print(" | Sonar : ");
  Serial.print(distanceSonar);

  Serial.println(" cm");
}

// setup

void setup()
{
  Serial.begin(115200);

  led.setpin(LEDPIN);
  led.setColor(0, 0, 0, 0);
  led.show();

  configEncodeurs();

  gyro.begin();
  gyro.resetData();

  Stop();

  debutEtat = millis();

  Serial.println("Labo 03 - DA 2421657");
  Serial.println("Attente de 3 secondes");
}

// loop

void loop()
{
  unsigned long currentTime = millis();

  encoderLeft.loop();
  encoderRight.loop();

  gyroTask(currentTime);
  stateTask(currentTime);
  ledTask(currentTime);
  serialTask(currentTime);
}
