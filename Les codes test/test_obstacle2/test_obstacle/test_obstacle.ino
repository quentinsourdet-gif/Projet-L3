#include <Wire.h>
#include <Servo.h>

// ======================================================
// SUIVEUR DE LIGNE
// ======================================================

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

// ======================================================
// MOTEURS
// ======================================================

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01
#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02
#define ARRET          0x00

// ======================================================
// SERVO + ULTRASON GROVE V2
// ======================================================

#define SERVO_PIN     3
#define ULTRASON_PIN  7

Servo monServo;

// ======================================================
// PID
// ======================================================

float Kp = 2.0;
float Kd = 0.5;

float erreur = 0;
float erreurPrec = 0;
float correction = 0;

#define VITESSE_BASE 28
#define VITESSE_MAX  40
#define VITESSE_MIN  15

int derniereErreur = 0;

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Wire.begin();

  Serial.begin(9600);

  monServo.attach(SERVO_PIN);

  monServo.write(90);

  delay(1000);

  Serial.println("ROBOT PRET");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  long distance = mesurerDistance();

  Serial.print("Distance : ");
  Serial.println(distance);

  // ==================================================
  // OBSTACLE
  // ==================================================

  if (distance > 0 && distance < 12)
  {
    stopRobot();

    Serial.println("OBSTACLE DETECTE");

    delay(500);

    // ==============================================
    // SCAN GAUCHE
    // ==============================================

    monServo.write(20);

    delay(700);

    long distanceGauche = mesurerDistance();

    // ==============================================
    // SCAN DROITE
    // ==============================================

    monServo.write(160);

    delay(700);

    long distanceDroite = mesurerDistance();

    // ==============================================
    // RETOUR CENTRE
    // ==============================================

    monServo.write(90);

    delay(500);

    Serial.print("Gauche : ");
    Serial.println(distanceGauche);

    Serial.print("Droite : ");
    Serial.println(distanceDroite);

    // ==============================================
    // CHOIX
    // ==============================================

    if (distanceGauche > distanceDroite)
    {
      eviterParGauche();
    }
    else
    {
      eviterParDroite();
    }

    return;
  }

  // ==================================================
  // SUIVI LIGNE
  // ==================================================

  suivreLignePID();
}

// ======================================================
// DISTANCE
// ======================================================

long mesurerDistance()
{
  pinMode(ULTRASON_PIN, OUTPUT);

  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(5);

  digitalWrite(ULTRASON_PIN, LOW);

  pinMode(ULTRASON_PIN, INPUT);

  long duree =
    pulseIn(ULTRASON_PIN,
            HIGH,
            30000);

  if (duree == 0)
    return 999;

  long distance = duree / 29 / 2;

  return distance;
}

// ======================================================
// CAPTEURS LIGNE
// ======================================================

uint8_t lireCapteurs()
{
  Wire.beginTransmission(ADDR_LF);

  Wire.write(REG_DIGITAL);

  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)ADDR_LF,
                   (uint8_t)1);

  if (!Wire.available())
    return 0xFF;

  return Wire.read() & 0x0F;
}

// ======================================================
// ERREUR
// ======================================================

int calculErreur(uint8_t e)
{
  switch(e)
  {
    case 0b1001: return 0;
    case 0b0110: return 0;

    case 0b0001: return -1;
    case 0b0011: return -2;
    case 0b0111: return -3;

    case 0b1000: return 1;
    case 0b1100: return 2;
    case 0b1110: return 3;
  }

  return derniereErreur;
}

// ======================================================
// SUIVI PID
// ======================================================

void suivreLignePID()
{
  uint8_t e = lireCapteurs();

  // ==================================================
  // LIGNE PERDUE
  // ==================================================

  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      piloterMoteur(MOTEUR_GAUCHE,
                    AVANT_GAUCHE,
                    15);

      piloterMoteur(MOTEUR_DROIT,
                    AVANT_DROIT,
                    28);
    }
    else
    {
      piloterMoteur(MOTEUR_GAUCHE,
                    AVANT_GAUCHE,
                    28);

      piloterMoteur(MOTEUR_DROIT,
                    AVANT_DROIT,
                    15);
    }

    return;
  }

  // ==================================================
  // GROS VIRAGES
  // ==================================================

  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  8);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  30);

    derniereErreur = -3;

    return;
  }

  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  30);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  8);

    derniereErreur = 3;

    return;
  }

  // ==================================================
  // PID
  // ==================================================

  erreur = calculErreur(e);

  derniereErreur = erreur;

  int P = erreur;

  int D = erreur - erreurPrec;

  correction = (Kp * P) + (Kd * D);

  erreurPrec = erreur;

  int vitesseGauche =
    VITESSE_BASE - correction;

  int vitesseDroite =
    VITESSE_BASE + correction;

  vitesseGauche =
    constrain(vitesseGauche,
              VITESSE_MIN,
              VITESSE_MAX);

  vitesseDroite =
    constrain(vitesseDroite,
              VITESSE_MIN,
              VITESSE_MAX);

  piloterMoteur(MOTEUR_GAUCHE,
                AVANT_GAUCHE,
                vitesseGauche);

  piloterMoteur(MOTEUR_DROIT,
                AVANT_DROIT,
                vitesseDroite);
}

// ======================================================
// EVITEMENT GAUCHE
// ======================================================

void eviterParGauche()
{
  stopRobot();

  delay(300);

  // TOURNE FORT

  tournerGauche();

  delay(850);

  // S'ELOIGNE

  avancer(24);

  delay(1500);

  // PARALLELE

  tournerDroite();

  delay(750);

  // LONGE L'OBSTACLE

  avancer(24);

  delay(2200);

  // REVIENT VERS LA LIGNE

  tournerDroite();

  delay(750);

  // CHERCHE LA LIGNE EN TOURNANT

  rechercherLigneGauche();
}

// ======================================================
// EVITEMENT DROITE
// ======================================================

void eviterParDroite()
{
  stopRobot();

  delay(300);

  // TOURNE FORT

  tournerDroite();

  delay(850);

  // S'ELOIGNE

  avancer(24);

  delay(1500);

  // PARALLELE

  tournerGauche();

  delay(750);

  // LONGE L'OBSTACLE

  avancer(24);

  delay(2200);

  // REVIENT VERS LA LIGNE

  tournerGauche();

  delay(750);

  // CHERCHE LA LIGNE EN TOURNANT

  rechercherLigneDroite();
}

// ======================================================
// RECHERCHE LIGNE GAUCHE
// ======================================================

void rechercherLigneGauche()
{
  while (1)
  {
    uint8_t e = lireCapteurs();

    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  12);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  24);

    if (e != 0b1111)
    {
      stopRobot();

      delay(300);

      break;
    }
  }
}

// ======================================================
// RECHERCHE LIGNE DROITE
// ======================================================

void rechercherLigneDroite()
{
  while (1)
  {
    uint8_t e = lireCapteurs();

    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  24);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  12);

    if (e != 0b1111)
    {
      stopRobot();

      delay(300);

      break;
    }
  }
}

// ======================================================
// MOUVEMENTS
// ======================================================

void avancer(int vitesse)
{
  piloterMoteur(MOTEUR_GAUCHE,
                AVANT_GAUCHE,
                vitesse);

  piloterMoteur(MOTEUR_DROIT,
                AVANT_DROIT,
                vitesse);
}

void tournerGauche()
{
  piloterMoteur(MOTEUR_GAUCHE,
                ARRIERE_GAUCHE,
                28);

  piloterMoteur(MOTEUR_DROIT,
                AVANT_DROIT,
                28);
}

void tournerDroite()
{
  piloterMoteur(MOTEUR_GAUCHE,
                AVANT_GAUCHE,
                28);

  piloterMoteur(MOTEUR_DROIT,
                ARRIERE_DROIT,
                28);
}

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE,
                ARRET,
                0);

  piloterMoteur(MOTEUR_DROIT,
                ARRET,
                0);
}

// ======================================================
// PILOTAGE MOTEURS
// ======================================================

void piloterMoteur(byte adresse,
                   byte direction,
                   byte vitesse)
{
  vitesse = constrain(vitesse, 0, 63);

  byte commande =
    (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);

  Wire.write(0x00);

  Wire.write(commande);

  Wire.endTransmission();
}