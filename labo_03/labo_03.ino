
#include <MeAuriga.h>
#include <Wire.h>

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeGyro gyro(0, 0x69);

const float X = 150.0;
const float diametreRoue = 6.5;
const float distanceObstacle = 30.0;
const int vitesse = 100;
const int vitessePivot = 50;

enum State
{
  WAITING,
  SEGMENT_1,
  PIVOT_1,
  SEGMENT_2,
  PIVOT_2,
  SEGMENT_3,
  PIVOT_180,
  FIN,
  RETOUR_3,
  RETOUR_PIVOT_1,
  RETOUR_2,
  RETOUR_PIVOT_2,
  RETOUR_1,
  FIN_BONUS
};

State currentState = WAITING;
unsigned long currentTime = 0;
unsigned long stateStartTime = 0;
long departGauche = 0;
long departDroite = 0;
bool premiereFois = true;
bool capteurValide = true;
float angleDepart = 0;
float distanceDebutSegment2 = 0;
float distanceSonar = 0;
float distanceSegment2 = 0;
bool segment1Termine = false;
bool segment2Termine = false;
bool segment3Termine = false;

// compter les impulsions de l'encodeur droit
void interruptionDroite()
{
  if (digitalRead(encoderRight.getPortB()) == 0)
    encoderRight.pulsePosMinus();
  else
    encoderRight.pulsePosPlus();
}

// compter les impulsions de l'encodeur gauche
void interruptionGauche()
{
  if (digitalRead(encoderLeft.getPortB()) == 0)
    encoderLeft.pulsePosMinus();
  else
    encoderLeft.pulsePosPlus();
}

// configurer les encodeurs et leurs regulateurs
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

// arreter les deux moteurs
void stopMoteurs()
{
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

// corriger la trajectoire droite avec le gyroscope
void avancerDroit(short speed = 100, short firstRun = 0)
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

// calculer la distance parcourue avec les encodeurs
float distanceParcourue()
{
  float gauche =
    abs(encoderLeft.getCurPos() - departGauche);

  float droite =
    abs(encoderRight.getCurPos() - departDroite);

  float moyenne = (gauche + droite) / 2.0;
  return moyenne * PI * diametreRoue / 360.0;
}

// memoriser la position de depart des roues
void initialiserDistance()
{
  departGauche = encoderLeft.getCurPos();
  departDroite = encoderRight.getCurPos();
}

// passer a l'etat suivant
void changerEtat(State nouvelEtat, unsigned long cT)
{
  stopMoteurs();
  currentState = nouvelEtat;
  stateStartTime = cT;
  premiereFois = true;
}

// attendre trois secondes avant le depart
void attenteTask(unsigned long cT)
{
  stopMoteurs();

  if (cT - stateStartTime >= 3000)
  {
    changerEtat(SEGMENT_1, cT);
  }
}

// segment 1 : avancer de 150 cm
void segment1Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    initialiserDistance();
    avancerDroit(vitesse, 1);
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
    changerEtat(PIVOT_1, cT);
    return;
  }

  avancerDroit(vitesse);
}

// effectuer un pivot selon la direction et l'angle demandes
bool pivoter(unsigned long cT, int direction, float angleCible)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    stopMoteurs();
    angleDepart = gyro.getAngleZ();
    premiereFois = false;
    lastTime = cT;
    return false;
  }

  if (cT - lastTime < 20)
    return false;

  lastTime = cT;

  float angleTourne =
    abs(gyro.getAngleZ() - angleDepart);

  if (angleTourne >= angleCible)
  {
    stopMoteurs();
    return true;
  }

  // direction 1 : droite, direction -1 : gauche
  encoderLeft.setMotorPwm(direction * vitessePivot);
  encoderRight.setMotorPwm(direction * vitessePivot);
  return false;
}

// premier pivot : tourner a droite de 90 degres
void pivotDroiteTask(unsigned long cT)
{
  if (pivoter(cT, 1, 90.0))
  {
    changerEtat(SEGMENT_2, cT);
  }
}

// segment 2 : avancer jusqu'a 30 cm de l'obstacle
void segment2Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    stopMoteurs();
    initialiserDistance();
    distanceDebutSegment2 = sonar.distanceCm();
    distanceSonar = distanceDebutSegment2;

    if (distanceSonar <= 0)
    {
      capteurValide = false;
      stopMoteurs();
      return;
    }

    if (distanceSonar <= distanceObstacle)
    {
      distanceSegment2 = 0;
      segment2Termine = true;
      changerEtat(PIVOT_2, cT);
      return;
    }

    avancerDroit(vitesse, 1);
    premiereFois = false;
    lastTime = cT;
    return;
  }

  if (cT - lastTime < 50)
    return;

  lastTime = cT;

  // arreter le robot si la mesure est invalide
  if (distanceSonar <= 0)
  {
    capteurValide = false;
    stopMoteurs();
    return;
  }

  // arreter a 30 cm et memoriser la distance parcourue
  if (distanceSonar <= distanceObstacle)
  {
    distanceSegment2 = distanceParcourue();
    segment2Termine = true;
    changerEtat(PIVOT_2, cT);
    return;
  }

  avancerDroit(vitesse);
}

// deuxieme pivot : tourner a gauche de 90 degres
void pivotGaucheTask(unsigned long cT)
{
  if (pivoter(cT, -1, 90.0))
  {
    changerEtat(SEGMENT_3, cT);
  }
}

// segment 3 : avancer de 150 cm
void segment3Task(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    initialiserDistance();
    avancerDroit(vitesse, 1);
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

  avancerDroit(vitesse);
}

// effectuer un demi-tour de 180 degres
void pivot180Task(unsigned long cT)
{
  if (pivoter(cT, 1, 180.0))
  {
    changerEtat(FIN, cT);
  }
}

// terminer le trajet aller avant le bonus
void finTask(unsigned long cT)
{
  stopMoteurs();

  // laisser les del clignoter pendant deux secondes
  if (cT - stateStartTime >= 2000)
  {
    changerEtat(RETOUR_3, cT);
  }
}

// avancer sur une distance precise pendant le retour
bool retourDistanceTask(unsigned long cT, float distanceCible)
{
  static unsigned long lastTime = 0;

  if (premiereFois)
  {
    initialiserDistance();
    avancerDroit(vitesse, 1);
    premiereFois = false;
    lastTime = cT;
    return false;
  }

  if (cT - lastTime < 20)
    return false;

  lastTime = cT;

  if (distanceParcourue() >= distanceCible)
  {
    stopMoteurs();
    return true;
  }

  avancerDroit(vitesse);
  return false;
}

// retour : parcourir les 150 cm du troisieme segment
void retour3Task(unsigned long cT)
{
  if (retourDistanceTask(cT, X))
  {
    changerEtat(RETOUR_PIVOT_1, cT);
  }
}

// retour : tourner a droite de 90 degres
void retourPivot1Task(unsigned long cT)
{
  if (pivoter(cT, 1, 90.0))
  {
    changerEtat(RETOUR_2, cT);
  }
}

// retour : parcourir la distance du deuxieme segment
void retour2Task(unsigned long cT)
{
  if (distanceSegment2 <= 0)
  {
    changerEtat(RETOUR_PIVOT_2, cT);
    return;
  }

  if (retourDistanceTask(cT, distanceSegment2))
  {
    changerEtat(RETOUR_PIVOT_2, cT);
  }
}

// retour : tourner a gauche de 90 degres
void retourPivot2Task(unsigned long cT)
{
  if (pivoter(cT, -1, 90.0))
  {
    changerEtat(RETOUR_1, cT);
  }
}

// retour : parcourir les 150 cm du premier segment
void retour1Task(unsigned long cT)
{
  if (retourDistanceTask(cT, X))
  {
    changerEtat(FIN_BONUS, cT);
  }
}

// immobiliser le robot a son point de depart
void finBonusTask(unsigned long cT)
{
  stopMoteurs();
}

// verifier si le robot effectue le trajet de retour
bool enRetour()
{
  return currentState == RETOUR_3 ||
         currentState == RETOUR_PIVOT_1 ||
         currentState == RETOUR_2 ||
         currentState == RETOUR_PIVOT_2 ||
         currentState == RETOUR_1;
}

// afficher un arc-en-ciel anime pendant le retour
void arcEnCielTask(unsigned long cT)
{
  const byte couleurs[6][3] = {
    {255, 0, 0},
    {255, 100, 0},
    {255, 255, 0},
    {0, 255, 0},
    {0, 0, 255},
    {150, 0, 255}
  };

  byte decalage = (cT / 100) % 6;

  for (int i = 1; i <= LEDNUM; i++)
  {
    byte index = (i - 1 + decalage) % 6;

    led.setColor(
      i,
      couleurs[index][0],
      couleurs[index][1],
      couleurs[index][2]
    );
  }
}

// gerer les etapes du parcours avec une machine a etats
void stateManager(unsigned long cT)
{
  if (!capteurValide)
  {
    stopMoteurs();
    return;
  }

  switch (currentState)
  {
    case WAITING:
      attenteTask(cT);
      break;

    case SEGMENT_1:
      segment1Task(cT);
      break;

    case PIVOT_1:
      pivotDroiteTask(cT);
      break;

    case SEGMENT_2:
      segment2Task(cT);
      break;

    case PIVOT_2:
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

    case RETOUR_3:
      retour3Task(cT);
      break;

    case RETOUR_PIVOT_1:
      retourPivot1Task(cT);
      break;

    case RETOUR_2:
      retour2Task(cT);
      break;

    case RETOUR_PIVOT_2:
      retourPivot2Task(cT);
      break;

    case RETOUR_1:
      retour1Task(cT);
      break;

    case FIN_BONUS:
      finBonusTask(cT);
      break;
  }
}

// mettre a jour les encodeurs des deux moteurs
void encoderTask(unsigned long cT)
{
  encoderLeft.loop();
  encoderRight.loop();
}

// mesurer la distance de l'obstacle pendant le segment 2
void sonarTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (currentState != SEGMENT_2 || cT - lastTime < 50)
    return;

  lastTime = cT;
  distanceSonar = sonar.distanceCm();
}

// actualiser l'angle du gyroscope toutes les 20 ms
void gyroTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 20)
    return;

  lastTime = cT;
  gyro.update();
}

// afficher la progression et les animations sur les del
void ledTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 50)
    return;

  lastTime = cT;
  led.setColor(0, 0, 0, 0);

  // progression des del 1 a 7 pendant le premier segment
  if (currentState == SEGMENT_1)
  {
    float progression =
      constrain(distanceParcourue() / X, 0.0, 1.0);

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
      led.setColor(i, 0, 255, 0);
  }

  // progression des del 1 a 7 pendant le deuxieme segment
  if (currentState == SEGMENT_2)
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

  // progression des del 1 a 7 pendant le troisieme segment
  if (currentState == SEGMENT_3)
  {
    float progression =
      constrain(distanceParcourue() / X, 0.0, 1.0);

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
      led.setColor(i, 0, 255, 0);
  }

  // garder les del vertes des segments termines
  if (segment1Termine)
    led.setColor(12, 0, 255, 0);

  if (segment2Termine)
    led.setColor(11, 0, 255, 0);

  if (segment3Termine)
    led.setColor(10, 0, 255, 0);

  // faire clignoter les del 8 a 12 en vert a 10 hz
  if (currentState == FIN)
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

  // afficher l'arc-en-ciel pendant le retour et a la fin
  if (enRetour() || currentState == FIN_BONUS)
  {
    arcEnCielTask(cT);
  }
  led.show();
}

// afficher les mesures dans le moniteur serie
void serialTask(unsigned long cT)
{
  static unsigned long lastTime = 0;

  if (cT - lastTime < 250)
    return;

  lastTime = cT;

  Serial.print("Etat : ");
  Serial.print(currentState);

  Serial.print(" | Distance : ");
  Serial.print(distanceParcourue());

  Serial.print(" cm | Angle Z : ");
  Serial.print(gyro.getAngleZ());

  Serial.print(" | Sonar : ");
  Serial.print(distanceSonar);

  Serial.println(" cm");
}

// initialiser les moteurs, les capteurs et les del
void setup()
{
  Serial.begin(115200);

  led.setpin(LEDPIN);
  led.setColor(0, 0, 0, 0);
  led.show();

  configEncodeurs();
  gyro.begin();
  gyro.resetData();
  stopMoteurs();

  stateStartTime = millis();

  Serial.println("Labo 03 - DA 2421657");
  Serial.println("Attente de 3 secondes");
}

// executer les taches et la machine a etats en continu
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