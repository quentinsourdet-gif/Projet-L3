#include <Wire.h>
#include <Servo.h>

// ======================================================
// SUIVEUR DE LIGNE
// ======================================================

#define ADDR_LF      0x20
#define REG_DIGITAL  0x07

// ======================================================
// MOTEURS
// ======================================================

#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01
#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02
#define ARRET          0x00

// ======================================================
// SERVO + ULTRASON
// ======================================================

#define SERVO_PIN    A3
#define ULTRASON_PIN 7

#define SERVO_CENTRE 90
#define SERVO_GAUCHE 0
#define SERVO_DROITE 180

Servo monServo;

// ======================================================
// PID
// ======================================================

float Kp = 2.0;
float Kd = 0.5;

float erreur     = 0;
float erreurPrec = 0;
float correction = 0;

#define VITESSE_BASE 28
#define VITESSE_MAX  40
#define VITESSE_MIN  15

int derniereErreur = 0;

// ======================================================
// PARAMETRES EVITEMENT
// ======================================================

#define DIST_ARRET              25
#define DIST_COTE_SEUIL         50
#define OBSTACLE_ACTUEL          1    // 1 = gauche | 2 = droite
#define DUREE_VIRAGE_90        900
#define TIMEOUT_RECHERCHE_LIGNE 6000

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);
  delay(1000);
  Serial.println("=== ROBOT PRET ===");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  long distance = mesurerDistance();

  Serial.print("Distance avant : ");
  Serial.println(distance);

  if (distance > 0 && distance < DIST_ARRET)
  {
    stopRobot();
    Serial.println(">>> OBSTACLE DETECTE <<<");
    delay(400);

    int angleScan = (OBSTACLE_ACTUEL == 1) ? SERVO_GAUCHE : SERVO_DROITE;

    eviterObstacle(angleScan);

    return;
  }

  suivreLignePID();
}

// ======================================================
// MESURE DISTANCE
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
  long duree = pulseIn(ULTRASON_PIN, HIGH, 30000);
  if (duree == 0) return 999;
  return duree / 29 / 2;
}

// ======================================================
// CAPTEURS LIGNE
// ======================================================

uint8_t lireCapteurs()
{
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0xFF;
  return Wire.read() & 0x0F;
}

// ======================================================
// CALCUL ERREUR
// ======================================================

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

// ======================================================
// SUIVI PID
// ======================================================

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

// ======================================================
// EVITEMENT
// ======================================================

void eviterObstacle(int angleScan)
{
  Serial.println("=== EVITEMENT ===");

  stopRobot();
  delay(300);

  // ------------------------------------------------
  // [1] Virage 90° initial
  // ------------------------------------------------
  Serial.println("[1] Virage 90deg");

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
  piloterMoteur(MOTEUR_DROIT,  ARRET,        0);
  delay(DUREE_VIRAGE_90);

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
  // [3] Rotation parallèle à l'obstacle
  // ------------------------------------------------
  Serial.println("[3] Rotation");

  piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  delay(900);

  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [4] Suivi obstacle (servo orienté côté)
  // ------------------------------------------------
  Serial.println("[4] Suivi obstacle");

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
    if (d < DIST_COTE_SEUIL) { obstacleVu = true; Serial.println("Obstacle detecte cote"); }
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
  // [5] Rotation retour vers la ligne
  // ------------------------------------------------
  Serial.println("[5] Rotation retour");

  piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);
  delay(900);

  stopRobot();
  delay(200);

  // ------------------------------------------------
  // [6] Retour ligne en LIGNE DROITE + réalignement
  // ------------------------------------------------
  Serial.println("[6] Recherche ligne");

  monServo.write(SERVO_CENTRE);
  delay(400);

  bool ligneFound = rechercherLigne();

  if (!ligneFound)
  {
    Serial.println("Ligne non trouvee");
    stopRobot();
  }

  Serial.println("=== EVITEMENT TERMINE ===");
}

// ======================================================
// RECHERCHE LIGNE — CORRIGÉ
// ======================================================

bool rechercherLigne()
{
  unsigned long debut = millis();

  // ---- PHASE 1 : avancer TOUT DROIT vers la ligne ----
  // Vitesses ÉGALES des deux côtés → trajectoire droite (bleu)
  // Avant : vitesses 12 et 24 → trajectoire courbe (rouge) ✗

  Serial.println("Phase 1 : avance lente vers la ligne");

  while (1)
  {
    if (millis() - debut > TIMEOUT_RECHERCHE_LIGNE)
    {
      Serial.println("Timeout");
      stopRobot();
      return false;
    }

    uint8_t e = lireCapteurs();

    // Vitesse réduite à 12 → robot plus lent = plus de temps pour détecter
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 16);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  16);
    
    delay(10); // petite pause pour laisser le temps au capteur de lire

    // S'arrête dès qu'un capteur voit autre chose que du blanc
    // 0b0000 = tous sur la ligne → inclus maintenant ✓
    if (e != 0b1111 && e != 0xFF)
    {
      // ---- LIGNE TROUVÉE — arrêt complet ----
      stopRobot();
      delay(500);
      Serial.println("Ligne trouvee ! Arret.");
      break;
    }
  }

  // ---- PHASE 2 : rotation GAUCHE pour reprendre le bon sens ----
  // Le robot arrive PERPENDICULAIRE à la ligne.
  // On tourne à gauche 90° pour reprendre le bon sens.

  Serial.println("Phase 2 : rotation gauche");

  piloterMoteur(MOTEUR_GAUCHE, ARRET,       0);   
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 28);   
  delay(DUREE_VIRAGE_90);

  stopRobot();
  delay(400);

  // Reset PID — repart proprement sans valeurs résiduelles
  erreur        = 0;
  erreurPrec    = 0;
  correction    = 0;
  derniereErreur = 0;

  Serial.println("Reprise suivi de ligne");

  return true;
}

// ======================================================
// MOUVEMENTS
// ======================================================

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

// ======================================================
// PILOTAGE MOTEURS I2C
// ======================================================

void piloterMoteur(byte adresse, byte direction, byte vitesse)
{
  vitesse = constrain(vitesse, 0, 63);
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}   