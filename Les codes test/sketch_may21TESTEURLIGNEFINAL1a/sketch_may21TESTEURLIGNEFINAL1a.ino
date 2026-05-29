#include <Wire.h>

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

#define AVANT_DROIT    0x02
#define AVANT_GAUCHE   0x01
#define ARRIERE_DROIT  0x01
#define ARRIERE_GAUCHE 0x02
#define ARRET          0x00

#define VIT_BASE        32
#define VIT_MAX         55
#define VIT_MIN          8
#define VIT_VIRAGE      28
#define VIT_VIRAGE_FORT 22

// ← NOUVEAU : nombre de cycles noir consécutifs avant arrêt
// Augmente cette valeur si il s'arrête encore trop tôt
#define SEUIL_ARRET     40

float Kp = 5.0;
float Ki = 0.2;
float Kd = 8.0;

float erreur_prec    = 0;
float integrale      = 0;
unsigned long t_prec = 0;
float derniere_pos   = 0;
int   compteur_noir  = 0;   // ← NOUVEAU : compte les cycles noir

void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

void arreter() {
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
}

void tournerGauche(byte vit) {
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,    vit);
  piloterMoteur(MOTEUR_GAUCHE, ARRIERE_GAUCHE, vit);
}

void tournerDroite(byte vit) {
  piloterMoteur(MOTEUR_DROIT,  ARRIERE_DROIT, vit);
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE,  vit);
}

uint8_t lireCapteurs() {
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0xFF;
  return Wire.read() & 0x0F;
}

float calculerPosition(uint8_t e) {
  float poids[4] = {-3.0, -1.0, 1.0, 3.0};
  float somme    = 0;
  int   nb       = 0;
  for (int i = 0; i < 4; i++) {
    if (e & (1 << i)) {
      somme += poids[i];
      nb++;
    }
  }
  if (nb == 0) return derniere_pos;
  float pos    = somme / nb;
  derniere_pos = pos;
  return pos;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(500);
  Serial.println("=== PID + virages + rampe ===");
  t_prec = millis();
}

void loop() {

  uint8_t e = lireCapteurs();

  // ── NOIR TOTAL → on compte les cycles ────────────────
  if (e == 0b0000) {
    compteur_noir++;
    Serial.print("NOIR x"); Serial.println(compteur_noir);

    if (compteur_noir >= SEUIL_ARRET) {
      // Noir confirmé longtemps = vraie ligne d'arrivée
      arreter();
      Serial.println("ARRET FINAL CONFIRME");
      while (true);
    } else {
      // Noir court = faux positif (rampe, ombre...)
      // On continue tout droit doucement
      piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  VIT_MIN);
      piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, VIT_MIN);
      return;
    }
  }

  // Dès qu'on voit autre chose que noir → reset compteur
  compteur_noir = 0;

  // ── BLANC TOTAL = ligne perdue ────────────────────────
  if (e == 0b1111) {
    if (derniere_pos > 0) tournerDroite(VIT_VIRAGE_FORT);
    else                  tournerGauche(VIT_VIRAGE_FORT);
    return;
  }

  // ── VIRAGE GAUCHE ─────────────────────────────────────
  if (e == 0b0111 || e == 0b0011 || e == 0b0001) {
    tournerGauche(VIT_VIRAGE);
    derniere_pos = -3.0;
    return;
  }

  // ── VIRAGE DROITE ─────────────────────────────────────
  if (e == 0b1110 || e == 0b1100 || e == 0b1000) {
    tournerDroite(VIT_VIRAGE);
    derniere_pos = 3.0;
    return;
  }

  // ── PID LIGNE DROITE ──────────────────────────────────
  float position = calculerPosition(e);

  unsigned long t_now = millis();
  float dt = (t_now - t_prec) / 1000.0;
  if (dt < 0.005) return;
  t_prec = t_now;

  float erreur   = position;
  integrale     += erreur * dt;
  integrale      = constrain(integrale, -10, 10);
  float derivee  = (erreur - erreur_prec) / dt;
  erreur_prec    = erreur;

  float correction = -(Kp * erreur + Ki * integrale + Kd * derivee);
  if (abs(correction) < 3.0) correction = 0;

  int vD = constrain((int)(VIT_BASE + correction), VIT_MIN, VIT_MAX);
  int vG = constrain((int)(VIT_BASE - correction), VIT_MIN, VIT_MAX);

  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT,  (byte)vD);
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, (byte)vG);

  Serial.print("e:"); Serial.print(e, BIN);
  Serial.print(" pos:"); Serial.print(position, 1);
  Serial.print(" cor:"); Serial.print(correction, 1);
  Serial.print(" vG:"); Serial.print(vG);
  Serial.print(" vD:"); Serial.println(vD);
}