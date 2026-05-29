/************************************************************
 * PROGRAMME PRINCIPAL FINAL STABLE
 * PID SIMPLE / OBSTACLES / SANS TUNNEL / REALIGNEMENT / SERVO A3
 * + TRIM_DROIT sur les pivots de l'evitement (roue droite)
 * + Detection couleur + affichage LED (nouvelle version)
 ************************************************************/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TCS34725.h>
#include <LiquidCrystal_I2C.h>

/* ==================== ADRESSES ==================== */

#define ADDR_LF        0x20
#define REG_DIGITAL    0x07

#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68

/* ==================== DIRECTIONS ==================== */

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01

#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02

#define ARRET          0x00

/* ==================== BROCHES ==================== */

#define SERVO_PIN      A3
#define ULTRASON_PIN   7
#define LED_PIN        6
#define LANCEUR_PIN    9

/* ==================== LED (ruban NeoPixel) ==================== */

#define NUM_LEDS    30
#define BRIGHTNESS  80

Adafruit_NeoPixel ruban(
  NUM_LEDS,
  LED_PIN,
  NEO_GRB + NEO_KHZ800
);

/* ==================== LCD ==================== */

LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ==================== CAPTEUR COULEUR ==================== */

#define COEF_R           1.20
#define COEF_G           1.05
#define COEF_B           1.05
#define SEUIL_CLEAR_MIN  5

Adafruit_TCS34725 tcs =
Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_240MS,
  TCS34725_GAIN_60X
);

/* ==================== SERVO ==================== */

Servo monServo;

#define SERVO_CENTRE 90
#define SERVO_GAUCHE 0
#define SERVO_DROITE 180

/* ==================== VITESSES ==================== */

#define VITESSE_BASE       28
#define VITESSE_MAX        40
#define VITESSE_MIN        15

#define VITESSE_EVITEMENT  24

/* ==================== TRIM PIVOTS EVITEMENT ====================
 * Applique UNIQUEMENT aux pivots de l'evitement ou la roue
 * DROITE est la roue qui tourne (28 -> 28 + TRIM_DROIT).
 *   pivots droite trop courts -> augmenter
 *   pivots droite trop longs  -> diminuer (peut etre negatif)
 * ============================================================ */

#define TRIM_DROIT  3

/* ==================== PID SIMPLE ==================== */

float Kp = 2.0;
float Kd = 0.5;

float erreur = 0;
float erreurPrec = 0;
float correction = 0;

int derniereErreur = 0;

/* ==================== DISTANCES ==================== */

#define DIST_ARRET            25
#define DIST_PANNEAU          5
#define DIST_COTE_OBSTACLE    50

/* ==================== TEMPS ==================== */

#define TIMEOUT_LIGNE         6000

/* ==================== EVITEMENT ==================== */

#define DUREE_VIRAGE_1      1200
#define DUREE_VIRAGE_3      1200
#define DUREE_VIRAGE_5      1200

#define DUREE_REALIGNEMENT  1200

/* ==================== PROTOTYPES ==================== */

String detecter_couleur();
void afficher_couleur_led(String couleur);

/* ==================== ETATS ==================== */

enum EtatRobot {

  SUIVI_O1,
  EVITEMENT_O1,

  SUIVI_O2,
  EVITEMENT_O2,

  SUIVI_COULEUR,
  COULEUR,

  DEMI_TOUR,
  RETOUR,

  FIN
};

EtatRobot etat = SUIVI_O1;

/* ==================================================== */

void setup()
{
  Wire.begin();

  Serial.begin(9600);

  monServo.attach(SERVO_PIN);

  monServo.write(SERVO_CENTRE);

  ruban.begin();

  ruban.setBrightness(BRIGHTNESS);

  ruban.clear();

  ruban.show();

  lcd.init();

  lcd.backlight();

  if (tcs.begin())
  {
    Serial.println("Capteur TCS34725 OK !");
  }
  else
  {
    Serial.println("ERREUR capteur couleur !");
  }

  Serial.println("Pret");
  delay(1000);
}

/* ==================================================== */

void loop()
{
  switch(etat)
  {

    case SUIVI_O1:
    {
      long d = mesurerDistance();

      if (d < DIST_ARRET)
      {
        stopRobot();

        etat = EVITEMENT_O1;
      }
      else
      {
        suivreLignePID();
      }

      break;
    }

    case EVITEMENT_O1:
    {
      eviterObstacle(true);

      etat = SUIVI_O2;

      break;
    }

    case SUIVI_O2:
    {
      long d = mesurerDistance();

      if (d < DIST_ARRET)
      {
        stopRobot();

        etat = EVITEMENT_O2;
      }
      else
      {
        suivreLignePID();
      }

      break;
    }

    case EVITEMENT_O2:
    {
      eviterObstacle(false);

      etat = SUIVI_COULEUR;

      break;
    }

    case SUIVI_COULEUR:
    {
      long d = mesurerDistance();

      if (d < DIST_PANNEAU)
      {
        stopRobot();

        etat = COULEUR;
      }
      else
      {
        suivreLignePID();
      }

      break;
    }

    case COULEUR:
    {
      String couleur = detecter_couleur();

      Serial.print(">>> Couleur : ");
      Serial.println(couleur);

      lcd.clear();
      lcd.print("Couleur:");
      lcd.setCursor(0, 1);
      lcd.print(couleur);

      if (couleur != "inconnue")
      {
        afficher_couleur_led(couleur);
      }
      else
      {
        delay(2000);
      }

      etat = DEMI_TOUR;

      break;
    }

    case DEMI_TOUR:
    {
      // RECUL

      piloterMoteur( MOTEUR_GAUCHE, ARRIERE_GAUCHE,25);
      piloterMoteur(MOTEUR_DROIT, ARRIERE_DROIT,25);
      delay(1600);
      stopRobot();
      delay(300);

      // ROTATION

      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE,30);
      piloterMoteur(MOTEUR_DROIT,ARRIERE_DROIT,30);
      delay(1800);
      stopRobot();
      delay(300);

      etat = RETOUR;

      break;
    }

    case RETOUR:
    {
      suivreLignePID();

      break;
    }

    case FIN:
    {
      stopRobot();

      break;
    }
  }
}

/* ==================================================== */
/* PID SIMPLE */
/* ==================================================== */

void suivreLignePID()
{
  uint8_t e = lireCapteurs();

  // LIGNE PERDUE

  if (e == 0b1111)
  {
    if (derniereErreur < 0)
    {
      avancerDiff(18, 30);
    }
    else
    {
      avancerDiff(30, 18);
    }

    return;
  }

  erreur = calculErreur(e);

  int P = erreur;

  int D = erreur - erreurPrec;

  correction = Kp * P + Kd * D;

  erreurPrec = erreur;

  derniereErreur = erreur;

  int vg =
  constrain(
    VITESSE_BASE - correction,
    VITESSE_MIN,
    VITESSE_MAX
  );

  int vd =
  constrain(
    VITESSE_BASE + correction,
    VITESSE_MIN,
    VITESSE_MAX
  );

  avancerDiff(vg, vd);
}

/* ==================================================== */

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

/* ==================================================== */
/* EVITEMENT FINALFINAL
 * Pivots : la roue DROITE qui tourne passe a 28 + TRIM_DROIT,
 * la roue GAUCHE reste a 28.
 * ==================================================== */

void eviterObstacle(bool gauche)
{
  stopRobot();

  delay(300);

  // VIRAGE 1

  if (gauche)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  28);

    piloterMoteur(MOTEUR_DROIT,
                  ARRET,
                  0);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  ARRET,
                  0);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  28 + TRIM_DROIT);
  }

  delay(DUREE_VIRAGE_1);

  stopRobot();

  delay(200);

  // AVANCE

  avancer(24);

  delay(3500);

  stopRobot();

  delay(200);

  // VIRAGE 2

  if (gauche)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  ARRET,
                  0);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  28 + TRIM_DROIT);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  28);

    piloterMoteur(MOTEUR_DROIT,
                  ARRET,
                  0);
  }

  delay(DUREE_VIRAGE_3);

  stopRobot();

  delay(200);

  // SUIVI OBSTACLE

  int angleScan =
  gauche ?
  SERVO_GAUCHE :
  SERVO_DROITE;

  monServo.write(angleScan);

  delay(400);

  while (1)
  {
    avancer(24);

    delay(80);

    long d = mesurerDistance();

    if (d > DIST_COTE_OBSTACLE)
    {
      delay(500);

      break;
    }
  }

  stopRobot();

  delay(200);

  // VIRAGE RETOUR

  if (gauche)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  ARRET,
                  0);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  28 + TRIM_DROIT);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  28);

    piloterMoteur(MOTEUR_DROIT,
                  ARRET,
                  0);
  }

  delay(DUREE_VIRAGE_5);

  stopRobot();

  delay(300);

  // REALIGNEMENT

  if (gauche)
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  ARRET,
                  0);

    piloterMoteur(MOTEUR_DROIT,
                  AVANT_DROIT,
                  28 + TRIM_DROIT);
  }
  else
  {
    piloterMoteur(MOTEUR_GAUCHE,
                  AVANT_GAUCHE,
                  28);

    piloterMoteur(MOTEUR_DROIT,
                  ARRET,
                  0);
  }

  delay(DUREE_REALIGNEMENT);

  stopRobot();

  delay(300);

  monServo.write(SERVO_CENTRE);

  delay(400);

  rechercherLigne(gauche);
}

/* ==================================================== */

bool rechercherLigne(bool gauche)
{
  unsigned long debut = millis();

  while (1)
  {
    if (millis() - debut >
        TIMEOUT_LIGNE)
    {
      stopRobot();

      return false;
    }

    uint8_t e = lireCapteurs();

    avancer(24);

    delay(10);

    if (e != 0b1111 &&
        e != 0xFF)
    {
      stopRobot();

      delay(500);

      break;
    }
  }

  erreur = 0;
  erreurPrec = 0;
  correction = 0;
  derniereErreur = 0;

  return true;
}

/* ====================================================
 * DETECTION COULEUR (version fournie, objet = tcs)
 * ==================================================== */

String detecter_couleur()
{
  uint16_t red, green, blue, clear;

  tcs.getRawData(&red, &green, &blue, &clear);

  Serial.print("BRUT R="); Serial.print(red);
  Serial.print(" G=");     Serial.print(green);
  Serial.print(" B=");     Serial.print(blue);
  Serial.print(" C=");     Serial.println(clear);

  if (clear < SEUIL_CLEAR_MIN)
  {
    Serial.println("-> Trop sombre");
    return "inconnue";
  }

  float r_norm = (float)red   / (float)clear;
  float g_norm = (float)green / (float)clear;
  float b_norm = (float)blue  / (float)clear;

  float r = r_norm * COEF_R;
  float g = g_norm * COEF_G;
  float b = b_norm * COEF_B;

  Serial.print("CORR R="); Serial.print(r, 3);
  Serial.print(" G=");     Serial.print(g, 3);
  Serial.print(" B=");     Serial.println(b, 3);

  float ratio_rg = r / g;
  float ratio_rb = r / b;
  float ratio_gr = g / r;
  float ratio_gb = g / b;
  float ratio_br = b / r;
  float ratio_bg = b / g;

  Serial.print("RATIOS R/G="); Serial.print(ratio_rg, 2);
  Serial.print(" R/B=");       Serial.print(ratio_rb, 2);
  Serial.print(" G/B=");       Serial.print(ratio_gb, 2);
  Serial.print(" B/G=");       Serial.println(ratio_bg, 2);

  // ROUGE
  if (ratio_rg > 1.15 && ratio_rb > 1.15)
  {
    return "rouge";
  }

  // VERT
  if (ratio_gr > 1.05 && ratio_gb > 1.05)
  {
    return "vert";
  }

  // BLEU
  if (ratio_br > 1.05 && ratio_bg > 1.05)
  {
    return "bleu";
  }

  return "inconnue";
}

/* ====================================================
 * AFFICHAGE COULEUR SUR LE RUBAN LED (version fournie)
 * ==================================================== */

void afficher_couleur_led(String couleur)
{
  uint32_t couleurLED;

  if (couleur == "rouge")
  {
    couleurLED = ruban.Color(255, 0, 0);
  }
  else if (couleur == "vert")
  {
    couleurLED = ruban.Color(0, 255, 0);
  }
  else if (couleur == "bleu")
  {
    couleurLED = ruban.Color(0, 0, 255);
  }
  else
  {
    couleurLED = ruban.Color(0, 0, 0);
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < NUM_LEDS; j++)
    {
      ruban.setPixelColor(j, couleurLED);
    }
    ruban.show();
    delay(500);

    ruban.clear();
    ruban.show();
    delay(500);
  }
}

/* ==================================================== */

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
  pulseIn(
    ULTRASON_PIN,
    HIGH,
    30000
  );

  if (duree == 0)
  {
    return 999;
  }

  return duree / 29 / 2;
}

/* ==================================================== */

uint8_t lireCapteurs()
{
  Wire.beginTransmission(ADDR_LF);

  Wire.write(REG_DIGITAL);

  Wire.endTransmission(false);

  Wire.requestFrom(
    (uint8_t)ADDR_LF,
    (uint8_t)1
  );

  if (!Wire.available())
  {
    return 0xFF;
  }

  return Wire.read() & 0x0F;
}

/* ==================================================== */
/* AVANCE TOUT DROIT (vitesses egales, pas de trim) */
/* ==================================================== */

void avancer(int vitesse)
{
  piloterMoteur(
    MOTEUR_GAUCHE,
    AVANT_GAUCHE,
    vitesse
  );

  piloterMoteur(
    MOTEUR_DROIT,
    AVANT_DROIT,
    vitesse
  );
}

/* ==================================================== */

void avancerDiff(int vg, int vd)
{
  vg = constrain(vg, 0, 63);

  vd = constrain(vd, 0, 63);

  piloterMoteur(
    MOTEUR_GAUCHE,
    AVANT_GAUCHE,
    vg
  );

  piloterMoteur(
    MOTEUR_DROIT,
    AVANT_DROIT,
    vd
  );
}

/* ==================================================== */

void stopRobot()
{
  piloterMoteur(
    MOTEUR_GAUCHE,
    ARRET,
    0
  );

  piloterMoteur(
    MOTEUR_DROIT,
    ARRET,
    0
  );
}

/* ==================================================== */

void piloterMoteur(
  byte adresse,
  byte direction,
  byte vitesse
)
{
  vitesse = constrain(vitesse, 0, 63);

  byte commande =
  (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);

  Wire.write(0x00);

  Wire.write(commande);

  Wire.endTransmission();
}
