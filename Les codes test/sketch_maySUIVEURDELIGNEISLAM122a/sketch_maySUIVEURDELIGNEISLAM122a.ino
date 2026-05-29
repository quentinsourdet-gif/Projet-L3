#include <Wire.h>

// ======================================================
// CAPTEUR SUIVEUR DE LIGNE
// ======================================================

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

// ======================================================
// MOTEURS
// ======================================================

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

// ======================================================
// DIRECTIONS
// ======================================================

#define AVANT_DROIT    0x02
#define ARRIERE_DROIT  0x01

#define AVANT_GAUCHE   0x01
#define ARRIERE_GAUCHE 0x02

#define ARRET          0x00

// ======================================================
// PID
// ======================================================

float Kp = 3.0;
float Kd = 0.4;

float erreur     = 0;
float erreurPrec = 0;
float correction = 0;

// ======================================================
// VITESSES
// ======================================================

#define VITESSE_BASE        30
#define VITESSE_BASE_VIRAGE 23

#define VITESSE_MAX   42
#define VITESSE_MIN   22

// ======================================================
// DERNIERE POSITION
// ======================================================

int derniereErreur = 0;

// ======================================================
// COMPTEUR ARRET FINAL
// << NOUVEAU : évite les faux arrêts
// ======================================================

int compteurNoir = 0;
#define SEUIL_ARRET 6   // doit voir 0000 pendant 6 cycles consécutifs

// ======================================================
// PILOTAGE MOTEUR
// ======================================================

void piloterMoteur(byte adresse,
                   byte direction,
                   byte vitesse)
{
  vitesse = constrain(vitesse, 0, 63);
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

// ======================================================
// STOP
// ======================================================

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
}

// ======================================================
// LECTURE CAPTEURS
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
// SETUP
// ======================================================

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("=== PID V3 ===");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  uint8_t e = lireCapteurs();

  // ==================================================
  // ARRET FINAL : 0000 confirmé N fois de suite
  // << CORRECTION : évite les faux arrêts prématurés
  // ==================================================

  if (e == 0b0000)
  {
    compteurNoir++;
    Serial.print("Noir détecté : ");
    Serial.println(compteurNoir);

    if (compteurNoir >= SEUIL_ARRET)
    {
      stopRobot();
      Serial.println("ARRET FINAL CONFIRME");
      while(true);
    }

    // continue à avancer doucement en attendant confirmation
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 24);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  24);
    delay(5);
    return;
  }
  else
  {
    // reset si ce n'était pas vraiment noir
    compteurNoir = 0;
  }

  // ==================================================
  // BLANC = ligne perdue (1111)
  // ==================================================

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

  // ==================================================
  // GROS VIRAGES SPECIAUX
  // << CORRECTION VIRAGE : valeurs encore plus basses
  // roue lente : 14→10 / roue rapide : 35→28
  // pour éviter de sortir par l'extérieur
  // ==================================================

  // gros virage gauche
  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 10); // << encore réduit
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  28); // << encore réduit
    derniereErreur = -3;
    return;
  }

  // gros virage droite
  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28); // << encore réduit
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  10); // << encore réduit
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

  static float correctionLisse = 0;
  float correctionBrute = (Kp * P) + (Kd * D);

  correctionLisse = 0.7 * correctionLisse + 0.3 * correctionBrute;
  correction = correctionLisse;

  if (abs(erreur) >= 2)
    correction *= 0.7;

  correction = constrain(correction, -8, 8);

  if (abs(correction) < 1.5)
    correction = 0;

  erreurPrec = erreur;

  // ==================================================
  // CALCUL VITESSES
  // ==================================================

  int vitesseBase = (abs(erreur) >= 2) ? VITESSE_BASE_VIRAGE : VITESSE_BASE;

  int vitesseGauche = vitesseBase - correction;
  int vitesseDroite = vitesseBase + correction;

  if (vitesseGauche < 22) vitesseGauche = 22;
  if (vitesseDroite < 22) vitesseDroite = 22;

  vitesseGauche = constrain(vitesseGauche, VITESSE_MIN, VITESSE_MAX);
  vitesseDroite = constrain(vitesseDroite, VITESSE_MIN, VITESSE_MAX);

  // ==================================================
  // ENVOI AUX MOTEURS
  // ==================================================

  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vitesseGauche);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  vitesseDroite);

  // ==================================================
  // DEBUG
  // ==================================================

  Serial.print("Capteurs : ");
  Serial.print(e, BIN);
  Serial.print("  erreur:");
  Serial.print(erreur);
  Serial.print("  corr:");
  Serial.print(correction);
  Serial.print("  VG:");
  Serial.print(vitesseGauche);
  Serial.print("  VD:");
  Serial.println(vitesseDroite);

  delay(5);
}