#include <Wire.h>

// ======================================================
// CAPTEUR SUIVEUR DE LIGNE
// ======================================================

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

// ======================================================
// ADRESSES MOTEURS
// ======================================================

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

// ======================================================
// DIRECTIONS MOTEURS
// (CORRIGE POUR MOTEURS SYMETRIQUES)
// ======================================================

#define AVANT_DROIT    0x02
#define AVANT_GAUCHE   0x02

#define ARRIERE_DROIT  0x01
#define ARRIERE_GAUCHE 0x01

#define ARRET          0x00

// ======================================================
// VITESSES
// ======================================================

#define VIT_BASE   30
#define VIT_MAX    45
#define VIT_MIN    10

// ======================================================
// PILOTAGE MOTEUR
// ======================================================

void piloterMoteur(byte adresse,
                   byte direction,
                   byte vitesse)
{
  if (vitesse > 63)
    vitesse = 63;

  byte commande = (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);

  Wire.write(0x00);
  Wire.write(commande);

  Wire.endTransmission();
}

// ======================================================
// STOP ROBOT
// ======================================================

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
// LECTURE CAPTEURS
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
// SETUP
// ======================================================

void setup()
{
  Wire.begin();

  Serial.begin(9600);

  Serial.println("=== SUIVEUR DE LIGNE ===");
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  uint8_t e = lireCapteurs();

  int vG = VIT_BASE;
  int vD = VIT_BASE;

  // ==================================================
  // LIGNE AU CENTRE
  // ==================================================

  if (e == 0b1001 || e == 0b0110)
  {
    vG = 30;
    vD = 30;
  }

  // ==================================================
  // PETIT VIRAGE GAUCHE
  // ==================================================

  else if (e == 0b0001)
  {
    vG = 15;
    vD = 38;
  }

  // ==================================================
  // GROS VIRAGE GAUCHE
  // ==================================================

  else if (e == 0b0011 || e == 0b0111)
  {
    vG = 10;
    vD = 42;
  }

  // ==================================================
  // PETIT VIRAGE DROITE
  // ==================================================

  else if (e == 0b1000)
  {
    vG = 38;
    vD = 15;
  }

  // ==================================================
  // GROS VIRAGE DROITE
  // ==================================================

  else if (e == 0b1100 || e == 0b1110)
  {
    vG = 42;
    vD = 10;
  }

  // ==================================================
  // BLANC TOTAL = LIGNE PERDUE
  // ==================================================

  else if (e == 0b1111)
  {
    stopRobot();

    Serial.println("BLANC");

    return;
  }

  // ==================================================
  // NOIR TOTAL = ARRIVEE
  // ==================================================

  else if (e == 0b0000)
  {
    stopRobot();

    Serial.println("NOIR");

    return;
  }

  // ==================================================
  // ENVOI AUX MOTEURS
  // ==================================================

  piloterMoteur(MOTEUR_GAUCHE,
                AVANT_GAUCHE,
                constrain(vG,
                          VIT_MIN,
                          VIT_MAX));

  piloterMoteur(MOTEUR_DROIT,
                AVANT_DROIT,
                constrain(vD,
                          VIT_MIN,
                          VIT_MAX));

  // ==================================================
  // DEBUG
  // ==================================================

  Serial.print("Capteurs : ");
  Serial.print(e, BIN);

  Serial.print(" | VG:");
  Serial.print(vG);

  Serial.print(" | VD:");
  Serial.println(vD);

  delay(10);
}