#include <Wire.h>
#include <Servo.h>

#define ADDR_LF      0x20
#define REG_DIGITAL  0x07

#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01
#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02
#define ARRET          0x00

#define SERVO_PIN    A3
#define ULTRASON_PIN 7

#define SERVO_CENTRE 90
#define SERVO_GAUCHE 0
#define SERVO_DROITE 180

Servo monServo;

float Kp = 2.0;
float Kd = 0.5;

float erreur      = 0;
float erreurPrec  = 0;
float correction  = 0;

#define VITESSE_BASE 28
#define VITESSE_MAX  40
#define VITESSE_MIN  15

int derniereErreur = 0;

// ======================================================
// PARAMÈTRES ÉVITEMENT
// ======================================================

#define DIST_ARRET               25
#define DIST_COTE_SEUIL          50
#define TIMEOUT_RECHERCHE_LIGNE  6000

// Durée de chaque virage — réglage indépendant
#define DUREE_VIRAGE_1      1200   // étape [1] virage initial
#define DUREE_VIRAGE_3      1200   // étape [3] rotation parallèle
#define DUREE_VIRAGE_5      1200   // étape [5] rotation retour
#define DUREE_REALIGNEMENT  1200   // rechercherLigne — réalignement final

int numeroObstacle = 0;

void eviterObstacle(bool parLaGauche);
bool rechercherLigne(bool parLaGauche);

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);
  delay(1000);
  Serial.println("=== ROBOT PRET ===");
}

void loop()
{
  long distance = mesurerDistance();

  if (distance > 0 && distance < DIST_ARRET)
  {
    stopRobot();
    numeroObstacle++;

    Serial.print(">>> OBSTACLE ");
    Serial.print(numeroObstacle);
    Serial.println(" DETECTE <<<");

    delay(400);

    bool parLaGauche = (numeroObstacle == 1);
    eviterObstacle(parLaGauche);
    return;
  }

  suivreLignePID();
}

long mesurerDistance()
{
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(ULTRASON_PIN, LOW);
  pinMode(ULTRASON_PIN, INPUT);
  long duree = pulseIn(ULTRASON_PIN, HIGH, 30000);
  if (duree == 0) return 999;
  return duree / 29 / 2;
}

uint8_t lireCapteurs()
{
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0xFF;
  return Wire.read() & 0x0F;
}

int calculErreur(uint8_t e)
{
  switch (e)
  {
    case 0b1001: return  0;
    case 0b0110: return  0;
    case 0b0001: return -1;
    case 0b0011: return -2;
    case 0b0111: return -3;
    case 0b1000: return  1;
    case 0b1100: return  2;
    case 0b1110: return  3;
  }
  return derniereErreur;
}

void suivreLignePID()
{
  uint8_t e = lireCapteurs();

  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 15);
      piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  28);
    }
    else
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
      piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  15);
    }
    return;
  }

  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE,  8);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  30);
    derniereErreur = -3;
    return;
  }

  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,   8);
    derniereErreur = 3;
    return;
  }

  erreur = calculErreur(e);
  derniereErreur = erreur;

  int P = erreur;
  int D = erreur - erreurPrec;
  correction = (Kp * P) + (Kd * D);
  erreurPrec = erreur;

  int vG = constrain((int)(VITESSE_BASE - correction), VITESSE_MIN, VITESSE_MAX);
  int vD = constrain((int)(VITESSE_BASE + correction), VITESSE_MIN, VITESSE_MAX);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vG);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  vD);
}

void eviterObstacle(bool parLaGauche)
{
  Serial.println("=== EVITEMENT ===");
  stopRobot();
  delay(300);

  // ------------------------------------------------
  // [1] Virage initial — DUREE_VIRAGE_1
  // ------------------------------------------------
  Serial.println("[1] Virage initial");

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT,  ARRET,        0);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  }
  delay(DUREE_VIRAGE_1);
  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [2] Avance
  // ------------------------------------------------
  Serial.println("[2] Avance");
  avancer(24);
  delay(3500);
  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [3] Rotation parallèle — DUREE_VIRAGE_3
  // ------------------------------------------------
  Serial.println("[3] Rotation parallele");

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT,  ARRET,        0);
  }
  delay(DUREE_VIRAGE_3);
  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [4] Suivi obstacle
  // ------------------------------------------------
  Serial.println("[4] Suivi obstacle");

  int angleScan = parLaGauche ? SERVO_GAUCHE : SERVO_DROITE;
  monServo.write(angleScan);
  delay(400);

  bool obstacleVu = false;
  int tentatives  = 0;

  while (!obstacleVu && tentatives < 40)
  {
    avancer(24);
    delay(100);
    long d = mesurerDistance();
    Serial.print("Distance cote : "); Serial.println(d);
    if (d < DIST_COTE_SEUIL)
    {
      obstacleVu = true;
      Serial.println("Obstacle detecte cote");
    }
    tentatives++;
  }

  while (1)
  {
    avancer(24);
    delay(80);
    long d = mesurerDistance();
    Serial.print("Distance cote : "); Serial.println(d);
    if (d > DIST_COTE_SEUIL)
    {
      Serial.println("Obstacle depasse");
      delay(500);
      break;
    }
  }

  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [5] Rotation retour — DUREE_VIRAGE_5
  // ------------------------------------------------
  Serial.println("[5] Rotation retour");

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT,  ARRET,        0);
  }
  delay(DUREE_VIRAGE_5);
  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [6] Retour sur la ligne
  // ------------------------------------------------
  Serial.println("[6] Recherche ligne");

  monServo.write(SERVO_CENTRE);
  delay(400);

  bool ligneFound = rechercherLigne(parLaGauche);

  if (!ligneFound)
  {
    Serial.println("Ligne non trouvee");
    stopRobot();
  }

  Serial.println("=== EVITEMENT TERMINE ===");
}

bool rechercherLigne(bool parLaGauche)
{
  unsigned long debut = millis();

  Serial.println("Phase 1 : avance vers la ligne");

  while (1)
  {
    if (millis() - debut > TIMEOUT_RECHERCHE_LIGNE)
    {
      Serial.println("Timeout");
      stopRobot();
      return false;
    }

    uint8_t e = lireCapteurs();

    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 16);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  16);

    delay(10);

    if (e != 0b1111 && e != 0xFF)
    {
      stopRobot();
      delay(500);
      Serial.println("Ligne trouvee ! Arret.");
      break;
    }
  }

  // ------------------------------------------------
  // Réalignement — DUREE_REALIGNEMENT
  // ------------------------------------------------
  Serial.println("Phase 2 : rotation realignement");

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT,  ARRET,        0);
  }
  delay(DUREE_REALIGNEMENT);

  stopRobot();
  delay(400);

  erreur         = 0;
  erreurPrec     = 0;
  correction     = 0;
  derniereErreur = 0;

  Serial.println("Reprise suivi ligne");
  return true;
}

void avancer(int vitesse)
{
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vitesse);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  vitesse);
}

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
}

void piloterMoteur(byte adresse, byte direction, byte vitesse)
{
  vitesse = constrain(vitesse, 0, 63);
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}