#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include "rgb_lcd.h"

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01
#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02
#define ARRET          0x00

float Kp_droit  = 1.5;
float Kp_virage = 3.0;
float Kd        = 0.3;

float erreur     = 0;
float erreurPrec = 0;
float correction = 0;

#define VITESSE_BASE        30
#define VITESSE_BASE_VIRAGE 23
#define VITESSE_MAX         42
#define VITESSE_MIN         22

int derniereErreur = 0;
int compteurNoir   = 0;
#define SEUIL_ARRET 3

bool enVirage        = false;
bool couleurDetectee = false;

#define ULTRASON_PIN   7
#define DISTANCE_ARRET 5

rgb_lcd lcd;

#define COEF_R 1.20
#define COEF_G 1.05
#define COEF_B 1.05
#define SEUIL_CLEAR_MIN 5

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_240MS,
  TCS34725_GAIN_1X
);

#define LED_PIN    6
#define NUM_LEDS   30
#define BRIGHTNESS 80

Adafruit_NeoPixel ruban(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ======================================================
// PILOTAGE MOTEUR
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
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
}

// ======================================================
// ULTRASONS
// ======================================================

long mesurer_distance()
{
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_PIN, LOW);
  pinMode(ULTRASON_PIN, INPUT);
  long duree = pulseIn(ULTRASON_PIN, HIGH, 30000);
  long distance = duree / 58;
  if (duree == 0 || distance > 400) return -1;
  return distance;
}

// ======================================================
// LECTURE CAPTEURS LIGNE
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

int calculErreur(uint8_t e)
{
  switch(e)
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
// DETECTION COULEUR
// ======================================================

String detecter_couleur()
{
  uint16_t red, green, blue, clear;
  colorSensor.getRawData(&red, &green, &blue, &clear);

  Serial.print("BRUT R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.print(blue);
  Serial.print(" C="); Serial.println(clear);

  if (clear < SEUIL_CLEAR_MIN) return "inconnue";

  float r = ((float)red   / clear) * COEF_R;
  float g = ((float)green / clear) * COEF_G;
  float b = ((float)blue  / clear) * COEF_B;

  Serial.print("RATIO R/G="); Serial.print(r/g, 2);
  Serial.print(" R/B="); Serial.print(r/b, 2);
  Serial.print(" G/R="); Serial.print(g/r, 2);
  Serial.print(" G/B="); Serial.print(g/b, 2);
  Serial.print(" B/R="); Serial.print(b/r, 2);
  Serial.print(" B/G="); Serial.println(b/g, 2);

  if (r/g > 1.15 && r/b > 1.15) return "rouge";
  if (g/r > 1.05 && g/b > 1.05) return "vert";
  if (b/r > 1.05 && b/g > 1.05) return "bleu";

  return "inconnue";
}

// ======================================================
// CLIGNOTEMENT LED 10 SECONDES
// ======================================================

void afficher_couleur_led(String couleur)
{
  uint32_t couleurLED;
  if      (couleur == "rouge") couleurLED = ruban.Color(255, 0,   0  );
  else if (couleur == "vert")  couleurLED = ruban.Color(0,   255, 0  );
  else if (couleur == "bleu")  couleurLED = ruban.Color(0,   0,   255);
  else                         couleurLED = ruban.Color(255, 255, 255);

  unsigned long debut = millis();
  while (millis() - debut < 10000)
  {
    for (int j = 0; j < NUM_LEDS; j++)
      ruban.setPixelColor(j, couleurLED);
    ruban.show();
    delay(500);
    ruban.clear();
    ruban.show();
    delay(500);
  }
  ruban.clear();
  ruban.show();
}

// ======================================================
// DEMI TOUR + RECHERCHE LIGNE
// << CORRECTION : après le demi-tour, tourne doucement
//    à droite jusqu'à retrouver la ligne
// ======================================================

void faireDemiTour()
{
  Serial.println("DEMI TOUR...");

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE,  30);
  piloterMoteur(MOTEUR_DROIT,  ARRIERE_DROIT, 30);
  delay(1100);
  stopRobot();
  delay(300);

  Serial.println("Recherche ligne...");

  // tourne doucement à droite max 3 secondes
  unsigned long debut = millis();
  while (millis() - debut < 3000)
  {
    uint8_t e = lireCapteurs();

    // ligne trouvée → stop et reprend
    if (e != 0b1111 && e != 0b0000 && e != 0xFF)
    {
      Serial.println("Ligne trouvee !");
      stopRobot();
      delay(100);
      return;
    }

    // tourne doucement à droite
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE,  22);
    piloterMoteur(MOTEUR_DROIT,  ARRIERE_DROIT, 22);
    delay(5);
  }

  stopRobot();
  delay(100);
  Serial.println("Ligne non trouvee !");
}

// ======================================================
// RESET PID
// ======================================================

void resetPID()
{
  erreur         = 0;
  erreurPrec     = 0;
  correction     = 0;
  derniereErreur = 0;
  compteurNoir   = 0;
  enVirage       = false;
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 255);
  lcd.setCursor(0, 0);
  lcd.print("Robot Ultimate");
  lcd.setCursor(0, 1);
  lcd.print("Initialisation");
  delay(1500);
  lcd.clear();

  ruban.begin();
  ruban.setBrightness(BRIGHTNESS);
  ruban.clear();
  ruban.show();

  if (colorSensor.begin())
    Serial.println("Capteur couleur OK");
  else { Serial.println("ERREUR couleur!"); while(1); }

  Serial.println("=== ROBOT COMPLET ===");
  delay(1000);
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  // ==================================================
  // ULTRASONS
  // ==================================================

  if (!couleurDetectee)
  {
    long distance = mesurer_distance();

    lcd.setCursor(0, 0);
    lcd.print("Distance:       ");
    lcd.setCursor(0, 1);
    if (distance == -1) lcd.print("Hors portee     ");
    else { lcd.print(distance); lcd.print(" cm          "); }

    if (distance != -1 && distance <= DISTANCE_ARRET)
    {
      stopRobot();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Couleur:");

      String couleur = detecter_couleur();
      lcd.setCursor(0, 1);
      lcd.print(couleur);
      delay(1000);

      Serial.print("COULEUR : ");
      Serial.println(couleur);

      afficher_couleur_led(couleur);
      faireDemiTour();
      resetPID();
      couleurDetectee = true;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Retour en cours");
      return;
    }
  }

  // ==================================================
  // SUIVI DE LIGNE
  // ==================================================

  uint8_t e = lireCapteurs();

  // ==================================================
  // ARRET FINAL
  // ==================================================

  bool quasiNoir = (e == 0b0000 ||
                    e == 0b0001 ||
                    e == 0b0010 ||
                    e == 0b0100 ||
                    e == 0b1000);

  if (quasiNoir)
  {
    compteurNoir++;
    if (compteurNoir >= SEUIL_ARRET)
    {
      stopRobot();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ARRET FINAL");
      Serial.println("ARRET FINAL");
      while(true);
    }
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 15);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  15);
    delay(5);
    return;
  }
  else { compteurNoir = 0; }

  // ligne perdue
  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 18);
      piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  30);
    }
    else
    {
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
      piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  18);
    }
    return;
  }

  // gros virages
  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 8);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  24);
    derniereErreur = -3;
    enVirage = true;
    return;
  }

  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  6);
    derniereErreur = 3;
    enVirage = true;
    return;
  }

  // PID
  erreur = calculErreur(e);
  derniereErreur = erreur;

  int P = erreur;
  int D = erreur - erreurPrec;

  static float correctionLisse = 0;

  if (enVirage)
  {
    correctionLisse = 0;
    erreurPrec      = 0;
    enVirage        = false;
  }

  float Kp = (abs(erreur) <= 1) ? Kp_droit : Kp_virage;
  float correctionBrute = (Kp * P) + (Kd * D);

  correctionLisse = 0.7 * correctionLisse + 0.3 * correctionBrute;
  correction = correctionLisse;

  if (abs(erreur) >= 2) correction *= 0.7;
  correction = constrain(correction, -8, 8);

  float zoneMorte = (abs(erreur) <= 1) ? 2.0 : 1.5;
  if (abs(correction) < zoneMorte) correction = 0;

  erreurPrec = erreur;

  int vitesseBase   = (abs(erreur) >= 2) ? VITESSE_BASE_VIRAGE : VITESSE_BASE;
  int vitesseGauche = vitesseBase - correction;
  int vitesseDroite = vitesseBase + correction;

  if (vitesseGauche < 22) vitesseGauche = 22;
  if (vitesseDroite < 22) vitesseDroite = 22;

  vitesseGauche = constrain(vitesseGauche, VITESSE_MIN, VITESSE_MAX);
  vitesseDroite = constrain(vitesseDroite, VITESSE_MIN, VITESSE_MAX);

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vitesseGauche);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  vitesseDroite);

  delay(5);
}