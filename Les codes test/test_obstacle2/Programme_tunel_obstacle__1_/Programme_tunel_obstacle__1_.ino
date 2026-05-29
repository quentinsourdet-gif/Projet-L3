#include <Wire.h>
#include <Servo.h>

// ======================================================
// CAPTEUR LIGNE
// ======================================================
#define CAPTEUR_LIGNE 0x20

// ======================================================
// MOTEURS
// ======================================================
#define MOTEUR_G  0x68
#define MOTEUR_D  0x66

#define AVANT   0x01
#define ARRIERE 0x02
#define FREIN   0x03

// ======================================================
// SERVO + ULTRASON
// ======================================================
#define ULTRASON_PIN 2
#define SERVO_PIN   17

#define SERVO_CENTRE 57
#define SERVO_DROITE  57
#define SERVO_GAUCHE 100

Servo monServo;

// ======================================================
// PID SUIVI DE LIGNE
// ======================================================
float Kp = 2.0;
float Kd = 0.5;

float erreur      = 0;
float erreurPrec  = 0;
float correction  = 0;

#define VITESSE_BASE         28
#define VITESSE_MAX          40
#define VITESSE_MIN          15
#define VITESSE_RETOUR_LIGNE 22

int derniereErreur = 0;

// ======================================================
// PARAMÈTRES ÉVITEMENT
// ======================================================
#define DIST_ARRET               20
#define DIST_COTE_SEUIL          50
#define TIMEOUT_RECHERCHE_LIGNE  6000

#define DUREE_VIRAGE_1      1200
#define DUREE_VIRAGE_3      1200
#define DUREE_VIRAGE_5      1000
#define DUREE_REALIGNEMENT  1200

int numeroObstacle = 0;

// ======================================================
// MACHINE À ÉTATS
// ======================================================
enum Section { SECTION_1, SECTION_2 };
Section section = SECTION_1;

unsigned long tempsLignePerdue = 0;
bool lignePerdue = false;

// ======================================================
// PROTOTYPES
// ======================================================
void eviterObstacle(bool parLaGauche);
bool rechercherLigne(bool parLaGauche);

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);
  arreter();
  delay(500);
  Serial.println("=== ROBOT PRET ===");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
  uint8_t state = lireCapteurs();

  switch (section)
  {
    case SECTION_1:
      gererSuiviLigne(state);
      break;

    case SECTION_2:
      gererTunnel(state);
      break;
  }
}

// ======================================================
// SECTION 1 — SUIVI DE LIGNE + ÉVITEMENT
// ======================================================
void gererSuiviLigne(uint8_t state)
{
  // Ligne perdue → passage tunnel
  if (state == 0b1111 || state == 0xFF)
  {
    if (!lignePerdue)
    {
      lignePerdue      = true;
      tempsLignePerdue = millis();
    }

    if (millis() - tempsLignePerdue > 500)
    {
      arreter();
      delay(100);
      monServo.write(SERVO_DROITE);
      delay(200);
      lignePerdue = false;
      section     = SECTION_2;
      Serial.println("=== PASSAGE TUNNEL ===");
      return;
    }

    suivreLignePID(state);
    return;
  }

  lignePerdue      = false;
  tempsLignePerdue = 0;

  // Détection obstacle (seulement si < 2 obstacles faits)
  if (numeroObstacle < 2)
  {
    long distance = mesurerDistance();
    if (distance > 0 && distance < DIST_ARRET)
    {
      arreter();
      numeroObstacle++;
      Serial.print(">>> OBSTACLE ");
      Serial.print(numeroObstacle);
      Serial.println(" DETECTE <<<");
      delay(400);
      bool parLaGauche = (numeroObstacle == 1);
      eviterObstacle(parLaGauche);
      return;
    }
  }

  suivreLignePID(state);
}

// ======================================================
// SECTION 2 — TUNNEL (suivi mur droit)
// ======================================================
void gererTunnel(uint8_t state)
{
  // Ligne retrouvée → sortie tunnel
  if (state != 0b1111 && state != 0xFF)
  {
    arreter();
    delay(150);
    monServo.write(SERVO_CENTRE);
    delay(200);
    erreur         = 0;
    erreurPrec     = 0;
    correction     = 0;
    derniereErreur = 0;
    lignePerdue    = false;
    section        = SECTION_1;
    Serial.println("=== SORTIE TUNNEL → REPRISE LIGNE ===");
    return;
  }

  // Pointer servo vers mur droit
  monServo.write(SERVO_DROITE);
  delay(50);

  long distD = mesurerDistance();

  Serial.print("Tunnel distD = ");
  Serial.println(distD);

  // Maintenir 26cm du mur droit
  if (distD < 24)
  {
    avancerDiff(8, 35);    // trop proche → s'éloigner
  }
  else if (distD > 28)
  {
    avancerDiff(35, 8);    // trop loin → se rapprocher
  }
  else
  {
    avancerDiff(23, 23);   // bonne distance → tout droit
  }
}

// ======================================================
// SUIVI DE LIGNE PID (depuis code obstacle)
// ======================================================
void suivreLignePID(uint8_t e)
{
  // Ligne perdue
  if (e == 0b1111 || e == 0xFF)
  {
    if (derniereErreur < 0)
      avancerDiff(15, 28);
    else
      avancerDiff(28, 15);
    return;
  }

  // Virage fort gauche
  if (e == 0b0011 || e == 0b0111)
  {
    avancerDiff(8, 30);
    derniereErreur = -3;
    return;
  }

  // Virage fort droite
  if (e == 0b1100 || e == 0b1110)
  {
    avancerDiff(30, 8);
    derniereErreur = 3;
    return;
  }

  erreur = calculErreur(e);
  derniereErreur = erreur;

  float P = erreur;
  float D = erreur - erreurPrec;
  correction = (Kp * P) + (Kd * D);
  erreurPrec = erreur;

  int vG = constrain((int)(VITESSE_BASE - correction), VITESSE_MIN, VITESSE_MAX);
  int vD = constrain((int)(VITESSE_BASE + correction), VITESSE_MIN, VITESSE_MAX);

  avancerDiff(vG, vD);
}

// ======================================================
// ÉVITEMENT OBSTACLES
// ======================================================
void eviterObstacle(bool parLaGauche)
{
  Serial.println("=== EVITEMENT ===");
  arreter();
  delay(300);

  // [1] Virage initial
  Serial.println("[1] Virage initial");
  if (parLaGauche)
  {
    // tourner gauche : moteur G stop, moteur D avant
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, AVANT, 28);
  }
  else
  {
    // tourner droite : moteur G arriere, moteur D stop
    piloterMoteur(MOTEUR_G, ARRIERE, 28);
    piloterMoteur(MOTEUR_D, FREIN,   0);
  }
  delay(DUREE_VIRAGE_1);
  arreter();
  delay(200);

  // [2] Avance
  Serial.println("[2] Avance");
  avancerDiff(24, 24);
  delay(3500);
  arreter();
  delay(200);

  // [3] Rotation parallèle
  Serial.println("[3] Rotation parallele");
  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_G, ARRIERE, 28);
    piloterMoteur(MOTEUR_D, FREIN,   0);
  }
  else
  {
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, AVANT, 28);
  }
  delay(DUREE_VIRAGE_3);
  arreter();
  delay(200);

  // [4] Suivi obstacle
  Serial.println("[4] Suivi obstacle");
  int angleScan = parLaGauche ? SERVO_GAUCHE : SERVO_DROITE;
  monServo.write(angleScan);
  delay(400);

  bool obstacleVu = false;
  int tentatives  = 0;

  while (!obstacleVu && tentatives < 40)
  {
    avancerDiff(24, 24);
    delay(100);
    long d = mesurerDistance();
    Serial.print("Distance cote : "); Serial.println(d);
    if (d < DIST_COTE_SEUIL) { obstacleVu = true; }
    tentatives++;
  }

  while (1)
  {
    avancerDiff(24, 24);
    delay(80);
    long d = mesurerDistance();
    if (d > DIST_COTE_SEUIL)
    {
      Serial.println("Obstacle depasse");
      delay(500);
      break;
    }
  }

  arreter();
  delay(200);

  // [5] Rotation retour
  Serial.println("[5] Rotation retour");
  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_G, ARRIERE, 28);
    piloterMoteur(MOTEUR_D, FREIN,   0);
  }
  else
  {
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, AVANT, 28);
  }
  delay(DUREE_VIRAGE_5);
  arreter();
  delay(200);

  // [6] Recherche ligne
  Serial.println("[6] Recherche ligne");
  monServo.write(SERVO_CENTRE);
  delay(400);

  bool ligneFound = rechercherLigne(parLaGauche);
  if (!ligneFound)
  {
    Serial.println("Ligne non trouvee");
    arreter();
  }

  Serial.println("=== EVITEMENT TERMINE ===");
}

// ======================================================
// RECHERCHE LIGNE
// ======================================================
bool rechercherLigne(bool parLaGauche)
{
  unsigned long debut = millis();

  while (1)
  {
    if (millis() - debut > TIMEOUT_RECHERCHE_LIGNE)
    {
      arreter();
      return false;
    }

    uint8_t e = lireCapteurs();

    avancerDiff(VITESSE_RETOUR_LIGNE, VITESSE_RETOUR_LIGNE);
    delay(10);

    if (e != 0b1111 && e != 0xFF)
    {
      arreter();
      delay(500);
      Serial.println("Ligne trouvee !");
      break;
    }
  }

  // Réalignement
  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, AVANT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_G, ARRIERE, 28);
    piloterMoteur(MOTEUR_D, FREIN,   0);
  }
  delay(DUREE_REALIGNEMENT);

  arreter();
  delay(400);

  erreur = 0; erreurPrec = 0; correction = 0; derniereErreur = 0;
  return true;
}

// ======================================================
// FONCTIONS COMMUNES
// ======================================================
uint8_t lireCapteurs()
{
  Wire.beginTransmission(CAPTEUR_LIGNE);
  Wire.write(0x07);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)CAPTEUR_LIGNE, (uint8_t)1);
  if (Wire.available()) return Wire.read() & 0x0F;
  return 0xFF;
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

long mesurerDistance()
{
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_PIN, LOW);
  pinMode(ULTRASON_PIN, INPUT);
  long duree = pulseIn(ULTRASON_PIN, HIGH, 25000);
  if (duree == 0) return 999;
  return duree / 58;
}

void avancerDiff(int vitG, int vitD)
{
  vitG = constrain(vitG, 0, 63);
  vitD = constrain(vitD, 0, 63);
  piloterMoteur(MOTEUR_G, AVANT,   vitG);
  piloterMoteur(MOTEUR_D, ARRIERE, vitD);
}

void arreter()
{
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
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