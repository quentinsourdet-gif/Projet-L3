#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include "rgb_lcd.h"

// ======================================================
// CONFIGURATION
// ======================================================

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

#define LED_PIN      6
#define NUM_LEDS     30

#define SERVO_CENTRE 90
#define SERVO_GAUCHE 0
#define SERVO_DROITE 180

// ======================================================
// OBJETS
// ======================================================

Servo monServo;

rgb_lcd lcd;

Adafruit_NeoPixel ruban(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_TCS34725 colorSensor(
  TCS34725_INTEGRATIONTIME_240MS,
  TCS34725_GAIN_1X
);

// ======================================================
// PID
// ======================================================

float Kp = 2.0;
float Kd = 0.5;

float erreur = 0;
float erreurPrec = 0;
float correction = 0;

int derniereErreur = 0;

// ======================================================
// VITESSES
// ======================================================

#define VITESSE_BASE         28
#define VITESSE_MAX          40
#define VITESSE_MIN          15
#define VITESSE_RETOUR_LIGNE 24
#define VITESSE_TUNNEL       20

// ======================================================
// DISTANCES
// ======================================================

#define DIST_ARRET              25
#define DIST_COTE_SEUIL         50
#define DIST_COULEUR            18
#define DIST_TUNNEL             12
#define TIMEOUT_RECHERCHE_LIGNE 6000

// ======================================================
// TEMPS EVITEMENT
// ======================================================

#define DUREE_VIRAGE_1      1200
#define DUREE_VIRAGE_3      1200
#define DUREE_VIRAGE_5      1000
#define DUREE_REALIGNEMENT  1200

// ======================================================
// VARIABLES
// ======================================================

int numeroObstacle = 0;

bool modeTunnel = false;

// ======================================================
// PROTOTYPES
// ======================================================

void eviterObstacle(bool parLaGauche);
bool rechercherLigne(bool parLaGauche);
void suivreLigneTunnel();

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Wire.begin();

  Serial.begin(9600);

  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);

  ruban.begin();
  ruban.show();

  lcd.begin(16, 2);

  if (!colorSensor.begin())
  {
    Serial.println("Capteur couleur absent");
  }

  couleurLED(0, 0, 20);

  lcd.setCursor(0, 0);
  lcd.print("ROBOT PRET");

  delay(1500);

  Serial.println("=== ROBOT PRET ===");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  long distance = mesurerDistance();

  // ==================================================
  // MODE TUNNEL
  // ==================================================

  if (distance > 0 && distance < DIST_TUNNEL)
  {
    modeTunnel = true;
  }
  else
  {
    modeTunnel = false;
  }

  if (modeTunnel)
  {
    couleurLED(20, 20, 20);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MODE TUNNEL");

    suivreLigneTunnel();

    return;
  }

  // ==================================================
  // EVITEMENT OBSTACLE
  // ==================================================

  if (distance > 0 && distance < DIST_ARRET)
  {
    couleurLED(20, 0, 0);

    stopRobot();

    numeroObstacle++;

    Serial.print(">>> OBSTACLE ");
    Serial.print(numeroObstacle);
    Serial.println(" DETECTE <<<");

    delay(400);

    bool parLaGauche = (numeroObstacle == 1);

    eviterObstacle(parLaGauche);

    couleurLED(0, 0, 20);

    return;
  }

  // ==================================================
  // DETECTION COULEUR
  // ==================================================

  if (distance > 0 && distance < DIST_COULEUR)
  {
    String couleur = detecterCouleur();

    Serial.print("Couleur : ");
    Serial.println(couleur);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(couleur);

    if (couleur == "rouge")
    {
      couleurLED(20, 0, 0);

      demiTour();
    }
    else if (couleur == "vert")
    {
      couleurLED(0, 20, 0);
    }
    else if (couleur == "bleu")
    {
      couleurLED(0, 0, 20);
    }
  }

  // ==================================================
  // SUIVI NORMAL
  // ==================================================

  couleurLED(0, 0, 20);

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

  long duree = pulseIn(ULTRASON_PIN, HIGH, 30000);

  if (duree == 0)
    return 999;

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

  if (!Wire.available())
    return 0xFF;

  return Wire.read() & 0x0F;
}

// ======================================================
// CALCUL ERREUR
// ======================================================

int calculErreur(uint8_t e)
{
  switch (e)
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
// SUIVI NORMAL PID
// ======================================================

void suivreLignePID()
{
  uint8_t e = lireCapteurs();

  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 15);
      piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);
    }
    else
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
      piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 15);
    }
    return;
  }

  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 8);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 30);

    derniereErreur = -3;
    return;
  }

  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 8);

    derniereErreur = 3;
    return;
  }

  erreur = calculErreur(e);

  derniereErreur = erreur;

  int P = erreur;
  int D = erreur - erreurPrec;

  correction = (Kp * P) + (Kd * D);

  erreurPrec = erreur;

  int vG = constrain((int)(VITESSE_BASE - correction),
                     VITESSE_MIN,
                     VITESSE_MAX);

  int vD = constrain((int)(VITESSE_BASE + correction),
                     VITESSE_MIN,
                     VITESSE_MAX);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vG);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, vD);
}

// ======================================================
// SUIVI TUNNEL
// ======================================================

void suivreLigneTunnel()
{
  uint8_t e = lireCapteurs();

  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 10);
      piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 24);
    }
    else
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 24);
      piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 10);
    }
    return;
  }

  erreur = calculErreur(e);

  derniereErreur = erreur;

  float P = erreur;
  float D = erreur - erreurPrec;

  correction = (Kp * P) + (Kd * D);

  erreurPrec = erreur;

  int vG = constrain((int)(VITESSE_TUNNEL - correction), 10, 30);
  int vD = constrain((int)(VITESSE_TUNNEL + correction), 10, 30);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vG);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, vD);
}

// ======================================================
// EVITEMENT OBSTACLE
// ======================================================

void eviterObstacle(bool parLaGauche)
{
  stopRobot();
  delay(300);

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT, ARRET, 0);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);
  }

  delay(DUREE_VIRAGE_1);

  stopRobot();
  delay(200);

  avancer(24);

  delay(3500);

  stopRobot();
  delay(200);

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT, ARRET, 0);
  }

  delay(DUREE_VIRAGE_3);

  stopRobot();
  delay(200);

  int angleScan = parLaGauche ? SERVO_GAUCHE : SERVO_DROITE;

  monServo.write(angleScan);

  delay(400);

  bool obstacleVu = false;

  int tentatives = 0;

  while (!obstacleVu && tentatives < 40)
  {
    avancer(24);

    delay(100);

    long d = mesurerDistance();

    if (d < DIST_COTE_SEUIL)
    {
      obstacleVu = true;
    }

    tentatives++;
  }

  while (1)
  {
    avancer(24);

    delay(80);

    long d = mesurerDistance();

    if (d > DIST_COTE_SEUIL)
    {
      delay(500);
      break;
    }
  }

  stopRobot();
  delay(200);

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT, ARRET, 0);
  }

  delay(DUREE_VIRAGE_5);

  stopRobot();
  delay(200);

  monServo.write(SERVO_CENTRE);

  delay(400);

  rechercherLigne(parLaGauche);
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
      stopRobot();
      return false;
    }

    uint8_t e = lireCapteurs();

    avancer(VITESSE_RETOUR_LIGNE);

    delay(10);

    if (e != 0b1111 && e != 0xFF)
    {
      stopRobot();

      delay(400);

      break;
    }
  }

  if (parLaGauche)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT, ARRET, 0);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
    piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);
  }

  delay(DUREE_REALIGNEMENT);

  stopRobot();

  erreur = 0;
  erreurPrec = 0;
  correction = 0;
  derniereErreur = 0;

  return true;
}

// ======================================================
// COULEUR
// ======================================================

String detecterCouleur()
{
  uint16_t r, g, b, c;

  colorSensor.getRawData(&r, &g, &b, &c);

  if (c < 5)
    return "inconnue";

  float rf = (float)r / c;
  float gf = (float)g / c;
  float bf = (float)b / c;

  if (rf > 0.45 && rf > gf && rf > bf)
    return "rouge";

  if (gf > 0.45 && gf > rf && gf > bf)
    return "vert";

  if (bf > 0.45 && bf > rf && bf > gf)
    return "bleu";

  return "inconnue";
}

// ======================================================
// DEMI TOUR
// ======================================================

void demiTour()
{
  stopRobot();
  delay(300);

  piloterMoteur(MOTEUR_GAUCHE, ARRIERE_GAUCHE, 25);
  piloterMoteur(MOTEUR_DROIT, ARRIERE_DROIT, 25);

  delay(400);

  stopRobot();
  delay(200);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
  piloterMoteur(MOTEUR_DROIT, ARRIERE_DROIT, 30);

  delay(1300);

  stopRobot();
  delay(300);
}

// ======================================================
// LEDS
// ======================================================

void couleurLED(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    ruban.setPixelColor(i, ruban.Color(r, g, b));
  }

  ruban.show();
}

// ======================================================
// MOTEURS
// ======================================================

void avancer(int vitesse)
{
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vitesse);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, vitesse);
}

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT, ARRET, 0);
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