
#include <MeAuriga.h>
#include <Wire.h>

#define LEDNUM 12
#define LEDPIN 44

// ========================================
// LABORATOIRE 03 - DA 2421657
// Deux derniers chiffres du DA : 57
// DA[6] = 7 (impair)
// Premier pivot : 90 degres a droite
// Deuxieme pivot : 90 degres a gauche
// ========================================

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);

MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeGyro gyro(0, 0x69);

const float X = 120.0;
const float diametreRoue = 6.5;

const int vitesse = 100;
const int vitesseSegment2 = 60;
const int vitessePivot = 50;

const float distanceObstacle = 30.0;

// ========================================
// MACHINE A ETATS
// ========================================

enum Etat
{
  ATTENTE,
  SEGMENT_1,
  PIVOT_DROITE,
  SEGMENT_2,
  FIN
};

Etat etat = ATTENTE;

unsigned long debutEtat = 0;

long departGauche = 0;
long departDroite = 0;

bool premiereFois = true;

float angleDepart = 0;
const float angleCible = 90.0;

unsigned long debutPivot = 0;

float distanceDebutSegment2 = 0;
float distanceSonar = 0;

bool obstacleDetecte = false;

// ========================================
// INTERRUPTIONS DES ENCODEURS
// ========================================

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

// ========================================
// CONFIGURATION DES ENCODEURS
// ========================================

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

// ========================================
// ARRET DES MOTEURS
// ========================================

void Stop()
{
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

// ========================================
// AVANCER DROIT AVEC LE GYROSCOPE
// ========================================

void goStraight(short speed = 100, short firstRun = 0)
{
  static double localGoal = 0.0;

  const double kp = 4.75;
  const double kd = 1.0;

  static double previousError = 0.0;

  if (firstRun)
  {
    gyro.resetData();

    localGoal = gyro.getAngleZ();
    previousError = 0;

    return;
  }

  double error = gyro.getAngleZ() - localGoal;

  double output =
    kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(-speed - output);
}

// ========================================
// CALCULER LA DISTANCE PARCOURUE
// ========================================

float distanceParcourue()
{
  float gauche = abs(
    encoderLeft.getCurPos() - departGauche
  );

  float droite = abs(
    encoderRight.getCurPos() - departDroite
  );

  float moyenne = (gauche + droite) / 2.0;

  return moyenne * PI * diametreRoue / 360.0;
}

// ========================================
// CHANGER D'ETAT
// ========================================

void changerEtat(Etat nouvelEtat, unsigned long cT)
{
  Stop();

  etat = nouvelEtat;
  debutEtat = cT;
  premiereFois = true;
}

// ========================================
// ETAPE 1 : ATTENDRE 3 SECONDES
// ========================================

void attenteTask(unsigned long cT)
{
  Stop();

  if (cT - debutEtat >= 3000)
  {
    changerEtat(SEGMENT_1, cT);
  }
}

// ========================================
// ETAPE 2 : AVANCER DE 120 CM
// ========================================

void segment1Task(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 20;

  if (premiereFois)
  {
    departGauche = encoderLeft.getCurPos();
    departDroite = encoderRight.getCurPos();

    goStraight(vitesse, 1);

    premiereFois = false;
    lastTime = cT;

    return;
  }

  if (cT - lastTime < rate)
    return;

  lastTime = cT;

  if (distanceParcourue() >= X)
  {
    changerEtat(PIVOT_DROITE, cT);
    return;
  }

  goStraight(vitesse);
}

// ========================================
// ETAPE 3 : PIVOT DE 90 DEGRES A DROITE
// ========================================

void pivotDroiteTask(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 20;

  if (premiereFois)
  {
    Stop();

    angleDepart = gyro.getAngleZ();
    debutPivot = cT;

    premiereFois = false;
    lastTime = cT;

    return;
  }

  if (cT - lastTime < rate)
    return;

  lastTime = cT;

  float angleActuel = gyro.getAngleZ();

  float angleTourne =
    abs(angleActuel - angleDepart);

  // Arret lorsque le pivot atteint 90 degres
  if (angleTourne >= angleCible)
  {
    changerEtat(SEGMENT_2, cT);
    return;
  }

  // Securite : ne pas continuer si le pivot echoue
  if (cT - debutPivot >= 5000)
  {
    changerEtat(FIN, cT);
    return;
  }

  encoderLeft.setMotorPwm(vitessePivot);
  encoderRight.setMotorPwm(vitessePivot);
}

// ========================================
// ETAPE 4 : SEGMENT 2 - ARRET A 30 CM
// ========================================

void segment2Task(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 50;

  if (premiereFois)
  {
    Stop();

    distanceDebutSegment2 = sonar.distanceCm();
    distanceSonar = distanceDebutSegment2;

    departGauche = encoderLeft.getCurPos();
    departDroite = encoderRight.getCurPos();

    obstacleDetecte = false;

    goStraight(vitesseSegment2, 1);

    premiereFois = false;
    lastTime = cT;

    return;
  }

  if (cT - lastTime < rate)
    return;

  lastTime = cT;

  distanceSonar = sonar.distanceCm();

  // Si l'obstacle est a 30 cm ou moins
  if (distanceSonar > 0 &&
      distanceSonar <= distanceObstacle)
  {
    obstacleDetecte = true;
    changerEtat(FIN, cT);
    return;
  }

  // Securite : ne pas avancer si le sonar
  // ne donne pas une mesure valide
  if (distanceSonar <= 0)
  {
    obstacleDetecte = false;
    changerEtat(FIN, cT);
    return;
  }

  goStraight(vitesseSegment2);
}

// ========================================
// FIN DU TEST
// ========================================

void finTask(unsigned long cT)
{
  Stop();
}

// ========================================
// MACHINE A ETATS
// ========================================

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

    case FIN:
      finTask(cT);
      break;
  }
}

// ========================================
// TACHE DU GYROSCOPE
// ========================================

void gyroTask(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 20;

  if (cT - lastTime < rate)
    return;

  lastTime = cT;

  gyro.update();
}

// ========================================
// TACHE DES DEL
// ========================================

void ledTask(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 50;

  if (cT - lastTime < rate)
    return;

  lastTime = cT;

  led.setColor(0, 0, 0, 0);

  // Etape 1 : DEL jaunes pendant 3 secondes
  if (etat == ATTENTE)
  {
    led.setColor(0, 20, 20, 0);
  }

  // Segment 1 : progression des DEL 1 a 7
  if (etat == SEGMENT_1)
  {
    float progression = distanceParcourue() / X;

    progression = constrain(
      progression, 0.0, 1.0
    );

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
    {
      led.setColor(i, 0, 255, 0);
    }
  }

  // DEL 12 reste verte apres le segment 1
  if (etat == PIVOT_DROITE ||
      etat == SEGMENT_2 ||
      etat == FIN)
  {
    led.setColor(12, 0, 255, 0);
  }

  // Segment 2 : nouvelle barre de progression
  if (etat == SEGMENT_2)
  {
    float distancePrevue =
      distanceDebutSegment2 - distanceObstacle;

    if (distancePrevue > 0)
    {
      float progression =
        distanceParcourue() / distancePrevue;

      progression = constrain(
        progression, 0.0, 1.0
      );

      int nombreLeds = progression * 7;

      for (int i = 1; i <= nombreLeds; i++)
      {
        led.setColor(i, 0, 255, 0);
      }
    }
  }

  // DEL 11 verte seulement si l'obstacle
  // a bien ete detecte a 30 cm
  if (etat == FIN && obstacleDetecte)
  {
    led.setColor(11, 0, 255, 0);
  }

  led.show();
}

// ========================================
// MONITEUR SERIE
// ========================================

void serialTask(unsigned long cT)
{
  static unsigned long lastTime = 0;
  const unsigned long rate = 250;

  if (cT - lastTime < rate)
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

// ========================================
// SETUP
// ========================================

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

// ========================================
// LOOP
// ========================================

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
