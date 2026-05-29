#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include "rgb_lcd.h"

// ======================================================
// CAPTEURS LIGNE
// ======================================================
#define ADDR_LF 0x20
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
// ULTRASON + SERVO
// ======================================================
#define ULTRASON_PIN 7
#define SERVO_PIN A3

#define DISTANCE_ARRET 8   // 👈 plus sensible (important)

#define SERVO_CENTRE 90

Servo monServo;

// ======================================================
// COULEUR
// ======================================================
Adafruit_TCS34725 colorSensor(
  TCS34725_INTEGRATIONTIME_240MS,
  TCS34725_GAIN_1X
);

// ======================================================
// LCD / LED
// ======================================================
rgb_lcd lcd;

#define LED_PIN 6
#define NUM_LEDS 30
Adafruit_NeoPixel ruban(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

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
// ETAT
// ======================================================
bool couleurDetectee = false;

// ======================================================
// MOTEURS
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

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT, ARRET, 0);
}

void avancer(int v)
{
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, v);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, v);
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
// DISTANCE
// ======================================================
long mesurerDistance()
{
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_PIN, LOW);

  pinMode(ULTRASON_PIN, INPUT);
  long t = pulseIn(ULTRASON_PIN, HIGH, 30000);

  if (t == 0) return -1;
  return t / 58;
}

// ======================================================
// COULEUR
// ======================================================
String detecterCouleur()
{
  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);

  if (c < 5) return "inconnue";

  float rf = (float)r / c;
  float gf = (float)g / c;
  float bf = (float)b / c;

  if (rf > 0.45 && rf > gf && rf > bf) return "rouge";
  if (gf > 0.45 && gf > rf && gf > bf) return "vert";
  if (bf > 0.45 && bf > rf && bf > gf) return "bleu";

  return "inconnue";
}

// ======================================================
// DEMI TOUR + MARCHE ARRIERE (CORRIGÉ)
// ======================================================
void demiTour()
{
  Serial.println("RECUL 300ms");
  piloterMoteur(MOTEUR_GAUCHE, ARRIERE_GAUCHE, 25);
  piloterMoteur(MOTEUR_DROIT, ARRIERE_DROIT, 25);
  delay(300);   // 👈 demandé

  stopRobot();
  delay(200);

  Serial.println("DEMI TOUR");

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
  piloterMoteur(MOTEUR_DROIT, ARRIERE_DROIT, 30);
  delay(1300);

  stopRobot();
}

// ======================================================
// SUIVI LIGNE (PID SIMPLE + ROBUSTE)
// ======================================================
void suivreLigne()
{
  uint8_t e = lireCapteurs();

  if (e == 0b1111 || e == 0xFF)
  {
    avancer(20);
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

  erreur = (e == 0b1001 || e == 0b0110) ? 0 : derniereErreur;
  float P = erreur;
  float D = erreur - erreurPrec;

  correction = (Kp * P) + (Kd * D);
  erreurPrec = erreur;

  int vG = constrain(28 - correction, 15, 40);
  int vD = constrain(28 + correction, 15, 40);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vG);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, vD);
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);

  if (!colorSensor.begin())
  {
    Serial.println("ERREUR capteur couleur");
    while (1);
  }

  Serial.println("ROBOT OK");
}

// ======================================================
// LOOP PRINCIPAL
// ======================================================
void loop()
{
  long d = mesurerDistance();

  // ==================================================
  // COULEUR + OBSTACLE
  // ==================================================
  if (!couleurDetectee && d > 0 && d < DISTANCE_ARRET)
  {
    stopRobot();
    delay(300);

    String couleur = detecterCouleur();
    Serial.println("COULEUR: " + couleur);

    if (couleur != "inconnue")
    {
      demiTour();
      couleurDetectee = true;
      return;
    }
  }

  // ==================================================
  // LIGNE
  // ==================================================
  suivreLigne();
}