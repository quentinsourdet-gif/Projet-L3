#include <Wire.h>

#include "Ultrasonic.h"

// ======================================================
// ADRESSES I2C
// ======================================================

#define MOTEUR_GAUCHE  0x66
#define MOTEUR_DROIT   0x68

// ======================================================
// REGISTRE MOTEUR
// ======================================================

#define REG_MOTEUR 0x00

// ======================================================
// COMMANDES
// ======================================================

#define STOP       0x00
#define AVANT      0x01
#define ARRIERE    0x02

// ======================================================
// ULTRASON
// ======================================================

// BROCHE D7

Ultrasonic ultrasonic(7);

// ======================================================
// SETUP
// ======================================================

void setup()
{
    Wire.begin();

    Serial.begin(9600);

    Serial.println("TEST EVITEMENT");
}

// ======================================================
// PILOTAGE MOTEUR
// ======================================================

void moteur(byte adresse, byte direction, int vitesse)
{
    vitesse = constrain(vitesse, 0, 63);

    byte commande = (vitesse << 2) | direction;

    Wire.beginTransmission(adresse);

    Wire.write(REG_MOTEUR);

    Wire.write(commande);

    Wire.endTransmission();
}

// ======================================================
// AVANCER
// ======================================================

void avancer()
{
    moteur(MOTEUR_GAUCHE, ARRIERE, 35);

    moteur(MOTEUR_DROIT, AVANT, 35);
}

// ======================================================
// STOP
// ======================================================

void stopRobot()
{
    moteur(MOTEUR_GAUCHE, STOP, 0);

    moteur(MOTEUR_DROIT, STOP, 0);
}

// ======================================================
// TOURNER GAUCHE
// ======================================================

void tournerGauche()
{
    moteur(MOTEUR_GAUCHE, AVANT, 30);

    moteur(MOTEUR_DROIT, AVANT, 30);
}

// ======================================================
// TOURNER DROITE
// ======================================================

void tournerDroite()
{
    moteur(MOTEUR_GAUCHE, ARRIERE, 30);

    moteur(MOTEUR_DROIT, ARRIERE, 30);
}

// ======================================================
// EVITER OBSTACLE GAUCHE
// ======================================================

void eviterObstacle()
{
    // stop

    stopRobot();

    delay(300);

    // tourne gauche

    tournerGauche();

    delay(500);

    // avance

    avancer();

    delay(900);

    // tourne droite

    tournerDroite();

    delay(500);

    // avance

    avancer();

    delay(1200);

    // tourne droite

    tournerDroite();

    delay(500);

    // avance

    avancer();

    delay(900);

    // remet axe ligne

    tournerGauche();

    delay(500);
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
    // lecture distance

    long distance =
        ultrasonic.MeasureInCentimeters();

    // affichage

    Serial.print("Distance : ");

    Serial.println(distance);

    // obstacle detecte

    if(distance > 0 && distance < 15)
    {
        eviterObstacle();
    }

    else
    {
        avancer();
    }

    delay(50);
}