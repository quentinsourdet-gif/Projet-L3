#include <Wire.h>

// ---- Capteur de ligne (ton code qui marche) ----
#define ADDR_LF 0x20
#define REG_DIGITAL 0x07

// ---- Driver moteur Mini I2C Motor Driver Grove ----
#define MOTOR_DRIVER_ADDR 0x0F

// ---- Vitesses (à ajuster selon ton robot) ----
#define VITESSE_NORMALE 80
#define VITESSE_VIRAGE 60
#define VITESSE_PIVOT 70

// ---- Variables globales ----
bool ligneFinDetectee = false;

// =====================================================
void setup() {
Wire.begin();
Serial.begin(9600);

Serial.println("=== Robot Ultimate - Suivi de ligne ===");
delay(1000);

Serial.println("Demarrage dans 2 sec...");
delay(2000);
}

// =====================================================
void loop() {
if (!ligneFinDetectee) {
suivre_ligne();
} else {
arreter_robot();
Serial.println(">>> LIGNE D'ARRIVEE - ROBOT ARRETE");
while (1); // Bloque le robot
}
}

// =====================================================
// FONCTION : lireCapteurs() - VERSION QUI MARCHE
// =====================================================
uint8_t lireCapteurs() {
Wire.beginTransmission(ADDR_LF);
Wire.write(REG_DIGITAL);
Wire.endTransmission(false);
Wire.requestFrom((uint8_t)ADDR_LF, (uint8_t)1);
if (!Wire.available()) return 0xFF;
return Wire.read() & 0x0F;
}

// =====================================================
// FONCTION : suivre_ligne()
// Logique adaptée à TON capteur (1 = blanc, 0 = noir)
// =====================================================
void suivre_ligne() {
uint8_t e = lireCapteurs();

Serial.print("e: ");
Serial.print(e, BIN);
Serial.print(" -> ");

switch (e) {

case 0b0110:
case 0b1001:
// CENTRE -> tout droit
Serial.println("CENTRE - avancer");
avancer(VITESSE_NORMALE, VITESSE_NORMALE);
break;

case 0b1000:
// Déviation droite légère -> corriger vers la gauche
Serial.println("dev droite - corr gauche");
avancer(VITESSE_VIRAGE, VITESSE_NORMALE);
break;

case 0b1100:
// Déviation droite forte -> tourner plus à gauche
Serial.println("dev droite forte - tourner gauche");
avancer(30, VITESSE_NORMALE);
break;

case 0b1110:
// Déviation droite très forte -> pivoter à gauche
Serial.println("dev droite TRES forte - pivot gauche");
avancer(10, VITESSE_NORMALE);
break;

case 0b0001:
// Déviation gauche légère -> corriger vers la droite
Serial.println("dev gauche - corr droite");
avancer(VITESSE_NORMALE, VITESSE_VIRAGE);
break;

case 0b0011:
// Déviation gauche forte -> tourner plus à droite
Serial.println("dev gauche forte - tourner droite");
avancer(VITESSE_NORMALE, 30);
break;

case 0b0111:
// Déviation gauche très forte -> pivoter à droite
Serial.println("dev gauche TRES forte - pivot droite");
avancer(VITESSE_NORMALE, 10);
break;

case 0b0000:
// NOIR TOTAL = ligne d'arrivée détectée !
Serial.println(">>> NOIR TOTAL - LIGNE D'ARRIVEE !");
ligneFinDetectee = true;
arreter_robot();
break;

case 0b1111:
// BLANC TOTAL = ligne perdue
Serial.println("BLANC TOTAL - ligne perdue, recherche...");
chercher_ligne();
break;

default:
// Autres cas (croisement, transition) -> avancer tout droit
Serial.println("autre - avancer");
avancer(VITESSE_NORMALE, VITESSE_NORMALE);
break;
}
}

// =====================================================
// FONCTION : chercher_ligne()
// Pivote pour retrouver la ligne quand on l'a perdue
// =====================================================
void chercher_ligne() {
// Pivote à gauche pendant 2 sec max
unsigned long debut = millis();

while (millis() - debut < 2000) {
pivot_gauche(VITESSE_PIVOT);

uint8_t e = lireCapteurs();
// Si on retrouve la ligne (au moins un capteur voit du noir)
if (e != 0b1111 && e != 0xFF) {
arreter_robot();
delay(50);
return;
}
delay(20);
}

// Sinon essayer à droite pendant 3 sec
debut = millis();
while (millis() - debut < 3000) {
pivot_droite(VITESSE_PIVOT);

uint8_t e = lireCapteurs();
if (e != 0b1111 && e != 0xFF) {
arreter_robot();
delay(50);
return;
}
delay(20);
}

// Toujours rien -> arrêt
arreter_robot();
}

// =====================================================
// FONCTIONS DE PILOTAGE MOTEUR
// =====================================================
void avancer(int vitG, int vitD) {
envoyer_moteur(vitG, vitD);
}

void pivot_gauche(int vitesse) {
envoyer_moteur(-vitesse, vitesse);
}

void pivot_droite(int vitesse) {
envoyer_moteur(vitesse, -vitesse);
}

void arreter_robot() {
envoyer_moteur(0, 0);
}

// =====================================================
// FONCTION : envoyer_moteur()
// Communication I2C avec le Grove Mini I2C Motor Driver
// =====================================================
void envoyer_moteur(int vitG, int vitD) {
vitG = constrain(vitG, -255, 255);
vitD = constrain(vitD, -255, 255);

byte sensG = (vitG >= 0) ? 0 : 1;
byte sensD = (vitD >= 0) ? 0 : 1;

byte valG = abs(vitG);
byte valD = abs(vitD);

Wire.beginTransmission(MOTOR_DRIVER_ADDR);
Wire.write(0x82);
Wire.write(valG);
Wire.write(valD);
Wire.endTransmission();

Wire.beginTransmission(MOTOR_DRIVER_ADDR);
Wire.write(0xAA);
Wire.write((sensG << 0) | (sensD << 1));
Wire.endTransmission();
}

