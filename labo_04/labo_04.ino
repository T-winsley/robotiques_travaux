#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>

#define LEDNUM 12
#define LEDPIN 44
#define NB_IR 5

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor sonar(PORT_10);
MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeGyro gyro(0, 0x69);
Adafruit_seesaw ss;

const float X = 150.0;
const float diametreRoue = 6.5;
const float distanceObstacle = 30.0;

const int vitesse = 100;
const int vitessePivot = 65;
const int vitessePivotLigne = 50;
const int vitesseLigne = 70;
const int vitesseRecherche = 40;

const unsigned long perteLigneMs = 150;
const unsigned long rechercheMaxMs = 2500;
const unsigned long pivotMaxMs = 6500;
const unsigned long calibrationMaxMs = 12000;

enum Etat {
  BOOT,
  ATTENTE,
  LAB3,
  CALIB,
  SUIVRE_LIGNE,
  CMD_INVALIDE,
  NON_CALIB
};

enum EtapeLab3 {
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

enum PhaseLigne {
  PID_LIGNE,
  PIVOT_LIGNE,
  RETROUVER_LIGNE
};

Etat etat = BOOT;
EtapeLab3 etapeLab3 = WAITING;
PhaseLigne phaseLigne = PID_LIGNE;

unsigned long currentTime = 0;
unsigned long stateStartTime = 0;
unsigned long etapeStartTime = 0;

bool premiereFois = true;
bool capteurValide = true;

long departGauche = 0;
long departDroite = 0;

float angleDepart = 0;
float rotationLab3 = 0;
float distanceDebutSegment2 = 0;
float distanceSonar = 0;
float distanceSegment2 = 0;

bool segment1Termine = false;
bool segment2Termine = false;
bool segment3Termine = false;

int ir[NB_IR];
int irMin[NB_IR];
int irMax[NB_IR];

bool capteursDisponibles = false;
bool estCalibre = false;

float dernierAngleCalibration = 0;
float rotationCalibration = 0;

float dernierAnglePivotLigne = 0;
float rotationPivotLigne = 0;

float erreurPrecedente = 0;
float correctionPrecedente = 0;

unsigned long dernierControleLigne = 0;
unsigned long dernierControleCalibration = 0;
unsigned long debutPerteLigne = 0;
unsigned long debutPivotLigne = 0;
unsigned long debutRechercheLigne = 0;
unsigned long ligneStableDepuis = 0;
unsigned long reprisePidDepuis = 0;

String serialMessage = "";
bool enteteFF = false;
bool paquetBluetooth = false;

void interruptionDroite() {
  if (digitalRead(encoderRight.getPortB()) == 0)
    encoderRight.pulsePosMinus();
  else
    encoderRight.pulsePosPlus();
}

void interruptionGauche() {
  if (digitalRead(encoderLeft.getPortB()) == 0)
    encoderLeft.pulsePosMinus();
  else
    encoderLeft.pulsePosPlus();
}

void configEncodeurs() {
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

// Arrete les moteurs
void stopMoteurs() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

// Avance droit avec le gyroscope
void avancerDroit(short speed = 100, short firstRun = 0) {
  static double objectif = 0;
  static double erreurPrecedenteGyro = 0;

  const double kp = 6.5;
  const double kd = 1.0;

  if (firstRun) {
    objectif = gyro.getAngleZ();
    erreurPrecedenteGyro = 0;
    return;
  }

  double erreur = gyro.getAngleZ() - objectif;

  double correction =
    kp * erreur +
    kd * (erreur - erreurPrecedenteGyro);

  erreurPrecedenteGyro = erreur;

  encoderLeft.setMotorPwm(
    constrain((int)(speed - correction), -255, 255)
  );

  encoderRight.setMotorPwm(
    constrain((int)(-speed - correction), -255, 255)
  );
}

// Point de depart pour la distance
void initialiserDistance() {
  departGauche = encoderLeft.getCurPos();
  departDroite = encoderRight.getCurPos();
}

// Distance parcourue
float distanceParcourue() {
  float gauche =
    abs(encoderLeft.getCurPos() - departGauche);

  float droite =
    abs(encoderRight.getCurPos() - departDroite);

  float moyenne = (gauche + droite) / 2.0;

  return moyenne * PI * diametreRoue / 360.0;
}

// Difference d'angle entre -180 et 180
float variationAngle(float actuel, float precedent) {
  float delta = actuel - precedent;

  if (delta > 180.0)
    delta -= 360.0;

  if (delta < -180.0)
    delta += 360.0;

  return delta;
}

// Nom de l'etat pour le port serie
const char* nomEtat(Etat e) {
  switch (e) {
    case BOOT: return "BOOT";
    case ATTENTE: return "ATTENTE";
    case LAB3: return "LAB3";
    case CALIB: return "CALIB";
    case SUIVRE_LIGNE: return "SUIVRE_LIGNE";
    case CMD_INVALIDE: return "CMD_INVALIDE";
    case NON_CALIB: return "NON_CALIB";
  }

  return "?";
}

// Change l'etat et l'affiche
void changerEtat(Etat nouvelEtat, unsigned long ct) {
  stopMoteurs();

  etat = nouvelEtat;
  stateStartTime = ct;

  phaseLigne = PID_LIGNE;
  erreurPrecedente = 0;
  correctionPrecedente = 0;
  debutPerteLigne = 0;
  ligneStableDepuis = 0;
  reprisePidDepuis = 0;

  Serial.print("Etat : ");
  Serial.println(nomEtat(etat));
}

// Change l'etape du labo 3
void changerEtapeLab3(EtapeLab3 nouvelle, unsigned long ct) {
  stopMoteurs();

  etapeLab3 = nouvelle;
  etapeStartTime = ct;
  premiereFois = true;

  Serial.print("Etape LAB3 : ");
  Serial.println((int)etapeLab3);
}

// Demarre le labo 3
void demarrerLab3(unsigned long ct) {
  segment1Termine = false;
  segment2Termine = false;
  segment3Termine = false;

  capteurValide = true;
  distanceSonar = 0;
  distanceDebutSegment2 = 0;
  distanceSegment2 = 0;

  changerEtapeLab3(WAITING, ct);
  changerEtat(LAB3, ct);

  Serial.println("Labo 03 : attente de 3 secondes");
}

// Pivote sur place 
bool pivoter(unsigned long ct, int direction, float angleCible) {
  static unsigned long precedent = 0;

  if (premiereFois) {
    stopMoteurs();

    angleDepart = gyro.getAngleZ();
    rotationLab3 = 0;

    premiereFois = false;
    precedent = ct;
    return false;
  }

  if (ct - precedent < 20)
    return false;

  precedent = ct;

  float angleActuel = gyro.getAngleZ();

  rotationLab3 += fabs(
    variationAngle(angleActuel, angleDepart)
  );

  angleDepart = angleActuel;

  if (rotationLab3 >= angleCible) {
    stopMoteurs();
    return true;
  }

  encoderLeft.setMotorPwm(direction * vitessePivot);
  encoderRight.setMotorPwm(direction * vitessePivot);

  return false;
}

// Avance jusqu'a la distance voulue
bool avancerDistance(unsigned long ct, float cible) {
  static unsigned long precedent = 0;

  if (premiereFois) {
    initialiserDistance();
    avancerDroit(vitesse, 1);

    premiereFois = false;
    precedent = ct;
    return false;
  }

  if (ct - precedent < 20)
    return false;

  precedent = ct;

  if (distanceParcourue() >= cible) {
    stopMoteurs();
    return true;
  }

  avancerDroit(vitesse);
  return false;
}

// Segment 1 : avancer de X cm
void segment1Task(unsigned long ct) {
  if (avancerDistance(ct, X)) {
    segment1Termine = true;
    changerEtapeLab3(PIVOT_1, ct);
  }
}

// Pivot a droite de 90 degres
void pivotDroiteTask(unsigned long ct) {
  if (pivoter(ct, 1, 90.0)) {
    changerEtapeLab3(SEGMENT_2, ct);
  }
}

// Segment 2 : avancer jusqu'a 30 cm de l'obstacle
void segment2Task(unsigned long ct) {
  static unsigned long precedent = 0;

  if (premiereFois) {
    stopMoteurs();
    initialiserDistance();

    distanceDebutSegment2 = sonar.distanceCm();
    distanceSonar = distanceDebutSegment2;

    if (distanceSonar <= 0) {
      capteurValide = false;
      return;
    }

    if (distanceSonar <= distanceObstacle) {
      distanceSegment2 = 0;
      segment2Termine = true;
      changerEtapeLab3(PIVOT_2, ct);
      return;
    }

    avancerDroit(vitesse, 1);
    premiereFois = false;
    precedent = ct;
    return;
  }

  if (ct - precedent < 50)
    return;

  precedent = ct;

  if (distanceSonar <= 0) {
    capteurValide = false;
    stopMoteurs();
    return;
  }

  if (distanceSonar <= distanceObstacle) {
    distanceSegment2 = distanceParcourue();
    segment2Termine = true;
    changerEtapeLab3(PIVOT_2, ct);
    return;
  }

  avancerDroit(vitesse);
}

// Pivot a gauche de 90 degres
void pivotGaucheTask(unsigned long ct) {
  if (pivoter(ct, -1, 90.0)) {
    changerEtapeLab3(SEGMENT_3, ct);
  }
}

// Segment 3 : avancer de X cm
void segment3Task(unsigned long ct) {
  if (avancerDistance(ct, X)) {
    segment3Termine = true;
    changerEtapeLab3(PIVOT_180, ct);
  }
}

// Demi-tour de 180 degres
void pivot180Task(unsigned long ct) {
  if (pivoter(ct, 1, 180.0)) {
    changerEtapeLab3(FIN, ct);
  }
}

// Fin du parcours
void finTask(unsigned long ct) {
  stopMoteurs();

  // Clignotement vert pendant 2 secondes.
  // Ensuite, lancement du retour bonus.
  if (ct - etapeStartTime >= 2000) {
    changerEtapeLab3(RETOUR_3, ct);
  }
}

void retour3Task(unsigned long ct) {
  if (avancerDistance(ct, X)) {
    changerEtapeLab3(RETOUR_PIVOT_1, ct);
  }
}

void retourPivot1Task(unsigned long ct) {
  if (pivoter(ct, 1, 90.0)) {
    changerEtapeLab3(RETOUR_2, ct);
  }
}

void retour2Task(unsigned long ct) {
  if (distanceSegment2 <= 0) {
    changerEtapeLab3(RETOUR_PIVOT_2, ct);
    return;
  }

  if (avancerDistance(ct, distanceSegment2)) {
    changerEtapeLab3(RETOUR_PIVOT_2, ct);
  }
}

void retourPivot2Task(unsigned long ct) {
  if (pivoter(ct, -1, 90.0)) {
    changerEtapeLab3(RETOUR_1, ct);
  }
}

void retour1Task(unsigned long ct) {
  if (avancerDistance(ct, X)) {
    changerEtapeLab3(FIN_BONUS, ct);
  }
}

void finBonusTask(unsigned long ct) {
  stopMoteurs();
}

// Parcours du labo 3
void lab3Task(unsigned long ct) {
  if (!capteurValide) {
    stopMoteurs();
    Serial.println("Sonar invalide - arret LAB3");
    changerEtat(ATTENTE, ct);
    return;
  }

  switch (etapeLab3) {
    case WAITING:
      stopMoteurs();

      if (ct - etapeStartTime >= 3000)
        changerEtapeLab3(SEGMENT_1, ct);

      break;

    case SEGMENT_1:
      segment1Task(ct);
      break;

    case PIVOT_1:
      pivotDroiteTask(ct);
      break;

    case SEGMENT_2:
      segment2Task(ct);
      break;

    case PIVOT_2:
      pivotGaucheTask(ct);
      break;

    case SEGMENT_3:
      segment3Task(ct);
      break;

    case PIVOT_180:
      pivot180Task(ct);
      break;

    case FIN:
      finTask(ct);
      break;

    case RETOUR_3:
      retour3Task(ct);
      break;

    case RETOUR_PIVOT_1:
      retourPivot1Task(ct);
      break;

    case RETOUR_2:
      retour2Task(ct);
      break;

    case RETOUR_PIVOT_2:
      retourPivot2Task(ct);
      break;

    case RETOUR_1:
      retour1Task(ct);
      break;

    case FIN_BONUS:
      finBonusTask(ct);
      break;
  }
}

// Lit les 5 capteurs
void lireLigne() {
  for (byte i = 0; i < NB_IR; i++) {
    ir[i] = ss.analogRead(i);
  }
}

// Demarre la calibration
void demarrerCalibration(unsigned long ct) {
  if (!capteursDisponibles) {
    Serial.println("LyneTracker non detecte");
    changerEtat(ATTENTE, ct);
    return;
  }

  estCalibre = false;

  for (byte i = 0; i < NB_IR; i++) {
    irMin[i] = 1023;
    irMax[i] = 0;
  }

  rotationCalibration = 0;
  dernierAngleCalibration = gyro.getAngleZ();
  dernierControleCalibration = ct;

  changerEtat(CALIB, ct);
}

// Pivote de 360 degres et releve le min et le max
void calibrationTask(unsigned long ct) {
  if (ct - dernierControleCalibration < 20)
    return;

  dernierControleCalibration = ct;

  lireLigne();

  for (byte i = 0; i < NB_IR; i++) {
    if (ir[i] < irMin[i])
      irMin[i] = ir[i];

    if (ir[i] > irMax[i])
      irMax[i] = ir[i];
  }

  float angleActuel = gyro.getAngleZ();

  rotationCalibration += fabs(
    variationAngle(angleActuel, dernierAngleCalibration)
  );

  dernierAngleCalibration = angleActuel;

  if (rotationCalibration >= 360.0) {
    stopMoteurs();

    estCalibre = true;

    for (byte i = 0; i < NB_IR; i++) {
      if (irMax[i] - irMin[i] < 10)
        estCalibre = false;

      Serial.print("Capteur ");
      Serial.print(i);

      Serial.print(" | Min : ");
      Serial.print(irMin[i]);

      Serial.print(" | Max : ");
      Serial.println(irMax[i]);
    }

    if (estCalibre)
      Serial.println("Calibration terminee");
    else
      Serial.println("Calibration insuffisante");

    changerEtat(ATTENTE, ct);
    return;
  }

  if (ct - stateStartTime >= calibrationMaxMs) {
    Serial.println("Calibration interrompue - securite");
    estCalibre = false;
    changerEtat(ATTENTE, ct);
    return;
  }

  encoderLeft.setMotorPwm(vitessePivotLigne);
  encoderRight.setMotorPwm(vitessePivotLigne);
}

// Position de la ligne
bool positionLigne(float &position) {
  lireLigne();

  float numerateur = 0;
  float denominateur = 0;

  for (byte i = 0; i < NB_IR; i++) {
    float plage = irMax[i] - irMin[i];

    if (plage < 1)
      continue;

    float normalise =
      (ir[i] - irMin[i]) * 1000.0 / plage;

    normalise = constrain(
      normalise,
      0.0,
      1000.0
    );

    // Hypothese a verifier sur le robot :
    // le noir produit une valeur plus faible.
    float noir = 1000.0 - normalise;

    if (noir < 150)
      noir = 0;

    numerateur +=
      noir * ((int)i - 2) * 1000.0;

    denominateur += noir;
  }

  if (denominateur < 200)
    return false;

  position = numerateur / denominateur;

  return true;
}

// Demarre le demi-tour
void debuterPivotLigne(unsigned long ct) {
  stopMoteurs();

  phaseLigne = PIVOT_LIGNE;

  rotationPivotLigne = 0;
  dernierAnglePivotLigne = gyro.getAngleZ();
  debutPivotLigne = ct;

  erreurPrecedente = 0;
  correctionPrecedente = 0;
  debutPerteLigne = 0;

  Serial.println("Fin de ligne - pivot 180");
}

// Suivi de ligne avec PID
void suivreLigneTask(unsigned long ct) {
  if (ct - dernierControleLigne < 20)
    return;

  dernierControleLigne = ct;

  // Phase 1 : pivot de 180 degres
  if (phaseLigne == PIVOT_LIGNE) {
    float angleActuel = gyro.getAngleZ();

    rotationPivotLigne += fabs(
      variationAngle(angleActuel, dernierAnglePivotLigne)
    );

    dernierAnglePivotLigne = angleActuel;

    if (rotationPivotLigne >= 180.0) {
      stopMoteurs();

      phaseLigne = RETROUVER_LIGNE;
      debutRechercheLigne = ct;
      ligneStableDepuis = 0;

      Serial.println("Demi-tour termine");
      return;
    }

    if (ct - debutPivotLigne >= pivotMaxMs) {
      Serial.println("Pivot interrompu - securite");
      changerEtat(ATTENTE, ct);
      return;
    }

    encoderLeft.setMotorPwm(vitessePivotLigne);
    encoderRight.setMotorPwm(vitessePivotLigne);
    return;
  }

  float position = 0;
  bool ligneDetectee = positionLigne(position);

  // Phase 2 : retrouver la ligne
  if (phaseLigne == RETROUVER_LIGNE) {
    if (ct - debutRechercheLigne >= rechercheMaxMs) {
      Serial.println("Ligne introuvable - arret");
      changerEtat(ATTENTE, ct);
      return;
    }

    if (ligneDetectee) {
      if (ligneStableDepuis == 0)
        ligneStableDepuis = ct;

      if (ct - ligneStableDepuis >= 120) {
        stopMoteurs();

        phaseLigne = PID_LIGNE;
        reprisePidDepuis = ct;
        debutPerteLigne = 0;

        erreurPrecedente = 0;
        correctionPrecedente = 0;

        Serial.println("Ligne retrouvee - reprise PID");
        return;
      }
    }
    else {
      ligneStableDepuis = 0;
    }

    encoderLeft.setMotorPwm(vitesseRecherche);
    encoderRight.setMotorPwm(-vitesseRecherche);
    return;
  }

  // Phase 3 : fin de ligne
  if (!ligneDetectee) {
    stopMoteurs();

    if (debutPerteLigne == 0)
      debutPerteLigne = ct;

    if (ct - debutPerteLigne >= perteLigneMs)
      debuterPivotLigne(ct);

    return;
  }

  debutPerteLigne = 0;

  // Phase 4 : regulateur pid
  float erreur = position;

  const float kp = 0.025;
  const float ki = 0.0;
  const float kd = 0.005;

  // Le terme integral est nul ici.
  float correction =
    kp * erreur +
    kd * (erreur - erreurPrecedente);

  erreurPrecedente = erreur;

  correction = constrain(
    correction,
    -35.0,
    35.0
  );

  // Limitation des changements brusques
  correction = constrain(
    correction,
    correctionPrecedente - 8.0,
    correctionPrecedente + 8.0
  );

  correctionPrecedente = correction;

  int vitesseActuelle = vitesseLigne;

  if (reprisePidDepuis != 0 &&
      ct - reprisePidDepuis < 500) {
    vitesseActuelle = vitesseRecherche;
  }
  else {
    reprisePidDepuis = 0;
  }

  encoderLeft.setMotorPwm(
    constrain(
      (int)(vitesseActuelle + correction),
      -255,
      255
    )
  );

  encoderRight.setMotorPwm(
    constrain(
      (int)(-vitesseActuelle + correction),
      -255,
      255
    )
  );
}

// Traite une commande recue
void traiterCommande(String cmd, unsigned long ct) {
  cmd.trim();
  cmd.toLowerCase();

  // Ignorer les messages techniques Bluetooth
  if (cmd.length() == 0 || cmd.charAt(0) == '+')
    return;

  Serial.print("cmd recu : ");
  Serial.println(cmd);

  // STOP est prioritaire dans tous les etats
  if (cmd == "stop") {
    changerEtat(ATTENTE, ct);
    return;
  }

  // Commande inconnue
  if (cmd != "lab3" &&
      cmd != "cal" &&
      cmd != "ligne") {

    Serial.println("cmd INCONNUE");
    changerEtat(CMD_INVALIDE, ct);
    return;
  }

  // Les autres commandes exigent ATTENTE
  if (etat != ATTENTE) {
    Serial.println("Robot occupe : envoyer stop");
    return;
  }

  if (cmd == "lab3") {
    demarrerLab3(ct);
  }
  else if (cmd == "cal") {
    demarrerCalibration(ct);
  }
  else if (cmd == "ligne") {
    if (!estCalibre) {
      changerEtat(NON_CALIB, ct);
    }
    else {
      changerEtat(SUIVRE_LIGNE, ct);
    }
  }
}

// Reception des commandes Bluetooth
void serialEvent() {
  while (Serial.available()) {
    byte c = Serial.read();

    // En-tete du protocole Makeblock
    if (c == 0xFF && serialMessage.length() == 0) {
      enteteFF = true;
      continue;
    }

    if (enteteFF) {
      enteteFF = false;

      if (c == 0x55) {
        paquetBluetooth = true;
        continue;
      }
    }

    // Fin de la commande
    if (c == '\r' || c == '\n') {
      if (serialMessage.length() > 0) {
        if (paquetBluetooth) {
          byte crc = 0xFF ^ 0x55;

          for (unsigned int i = 0;
               i + 1 < serialMessage.length();
               i++) {
            crc ^= (byte)serialMessage[i];
          }

          byte crcRecu =
            (byte)serialMessage[
              serialMessage.length() - 1
            ];

          if (crc == crcRecu) {
            serialMessage.remove(
              serialMessage.length() - 1
            );

            traiterCommande(
              serialMessage,
              millis()
            );
          }
          else {
            Serial.println("CRC invalide");
          }
        }
        else {
          traiterCommande(
            serialMessage,
            millis()
          );
        }
      }

      serialMessage = "";
      paquetBluetooth = false;
      enteteFF = false;
    }
    else if (serialMessage.length() < 32) {
      serialMessage += (char)c;
    }
    else {
      serialMessage = "";
      paquetBluetooth = false;
      enteteFF = false;
    }
  }
}

void encoderTask(unsigned long ct) {
  encoderLeft.loop();
  encoderRight.loop();
}

void gyroTask(unsigned long ct) {
  static unsigned long precedent = 0;

  if (ct - precedent < 20)
    return;

  precedent = ct;
  gyro.update();
}

void sonarTask(unsigned long ct) {
  static unsigned long precedent = 0;

  if (etat != LAB3 || etapeLab3 != SEGMENT_2)
    return;

  if (ct - precedent < 50)
    return;

  precedent = ct;
  distanceSonar = sonar.distanceCm();
}

bool enRetour() {
  return etapeLab3 == RETOUR_3 ||
         etapeLab3 == RETOUR_PIVOT_1 ||
         etapeLab3 == RETOUR_2 ||
         etapeLab3 == RETOUR_PIVOT_2 ||
         etapeLab3 == RETOUR_1;
}

// Arc-en-ciel (bonus)
void arcEnCielTask(unsigned long ct) {
  const byte couleurs[6][3] = {
    {255, 0, 0},
    {255, 100, 0},
    {255, 255, 0},
    {0, 255, 0},
    {0, 0, 255},
    {150, 0, 255}
  };

  byte decalage = (ct / 100) % 6;

  for (int i = 1; i <= LEDNUM; i++) {
    byte index = (i - 1 + decalage) % 6;

    led.setColor(
      i,
      couleurs[index][0],
      couleurs[index][1],
      couleurs[index][2]
    );
  }
}

// DEL du labo 3
void ledLab3Task(unsigned long ct) {
  float progression = 0;

  if (etapeLab3 == SEGMENT_1 ||
      etapeLab3 == SEGMENT_3) {

    progression = constrain(
      distanceParcourue() / X,
      0.0,
      1.0
    );
  }

  if (etapeLab3 == SEGMENT_2) {
    float distancePrevue =
      distanceDebutSegment2 - distanceObstacle;

    if (distancePrevue > 0) {
      progression = constrain(
        distanceParcourue() / distancePrevue,
        0.0,
        1.0
      );
    }
  }

  if (etapeLab3 == SEGMENT_1 ||
      etapeLab3 == SEGMENT_2 ||
      etapeLab3 == SEGMENT_3) {

    int nombreLeds = progression * 7;

    for (int i = 1; i <= nombreLeds; i++)
      led.setColor(i, 0, 255, 0);
  }

  if (segment1Termine)
    led.setColor(12, 0, 255, 0);

  if (segment2Termine)
    led.setColor(11, 0, 255, 0);

  if (segment3Termine)
    led.setColor(10, 0, 255, 0);

  if (etapeLab3 == FIN) {
    bool allume = (ct / 50) % 2 == 0;

    for (int i = 8; i <= 12; i++) {
      if (allume)
        led.setColor(i, 0, 255, 0);
      else
        led.setColor(i, 0, 0, 0);
    }
  }

  if (enRetour() || etapeLab3 == FIN_BONUS)
    arcEnCielTask(ct);
}

// Gestion des DEL
void ledTask(unsigned long ct) {
  static unsigned long precedent = 0;

  if (ct - precedent < 50)
    return;

  precedent = ct;

  // Effacer les 12 DEL avant chaque affichage
  for (int i = 1; i <= LEDNUM; i++)
    led.setColor(i, 0, 0, 0);

  if (etat == LAB3) {
    ledLab3Task(ct);
  }

  // Commande invalide : rouge pendant 3 s
  if (etat == CMD_INVALIDE) {
    if ((ct / 250) % 2 == 0) {
      for (int i = 1; i <= LEDNUM; i++)
        led.setColor(i, 255, 0, 0);
    }
  }

  // Non calibre : jaune pendant 3 s
  if (etat == NON_CALIB) {
    if ((ct / 250) % 2 == 0) {
      for (int i = 1; i <= LEDNUM; i++)
        led.setColor(i, 255, 255, 0);
    }
  }

  led.show();
}

// Affichage sur le port serie
void serialTask(unsigned long ct) {
  static unsigned long precedent = 0;

  if (etat != LAB3)
    return;

  if (ct - precedent < 250)
    return;

  precedent = ct;

  Serial.print("Etape : ");
  Serial.print((int)etapeLab3);

  Serial.print(" | Distance : ");
  Serial.print(distanceParcourue());

  Serial.print(" cm | Angle Z : ");
  Serial.print(gyro.getAngleZ());

  Serial.print(" | Sonar : ");
  Serial.print(distanceSonar);

  Serial.println(" cm");
}

// Machine a etats principale
void stateManager(unsigned long ct) {
  switch (etat) {
    case BOOT:
      stopMoteurs();

      if (ct - stateStartTime >= 3000)
        changerEtat(ATTENTE, ct);

      break;

    case ATTENTE:
      stopMoteurs();
      break;

    case LAB3:
      lab3Task(ct);
      break;

    case CALIB:
      calibrationTask(ct);
      break;

    case SUIVRE_LIGNE:
      suivreLigneTask(ct);
      break;

    case CMD_INVALIDE:
      stopMoteurs();

      if (ct - stateStartTime >= 3000)
        changerEtat(ATTENTE, ct);

      break;

    case NON_CALIB:
      stopMoteurs();

      if (ct - stateStartTime >= 3000)
        changerEtat(ATTENTE, ct);

      break;
  }
}

void setup() {
  Serial.begin(115200);

  led.setpin(LEDPIN);
  led.setColor(0, 0, 0, 0);
  led.show();

  configEncodeurs();

  gyro.begin();
  gyro.resetData();

  capteursDisponibles = ss.begin();

  stopMoteurs();

  stateStartTime = millis();

  Serial.println("Laboratoire 04 - LAB3 integre");

  if (capteursDisponibles)
    Serial.println("LyneTracker connecte");
  else
    Serial.println("LyneTracker non detecte");

  Serial.println("Etat : BOOT");
}

void loop() {
  currentTime = millis();

  encoderTask(currentTime);
  gyroTask(currentTime);
  sonarTask(currentTime);

  stateManager(currentTime);

  ledTask(currentTime);
  serialTask(currentTime);
}