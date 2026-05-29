#include <Wire.h>
#include <Servo.h>
#include "Ultrasonic.h"

// =====================================
// MOTEURS
// =====================================

#define MOTEUR_A 0x66
#define MOTEUR_B 0x68

#define ARRET   0x00
#define AVANT   0x01
#define ARRIERE 0x02

#define VITESSE 50

// =====================================
// ULTRASON GROVE
// =====================================

Ultrasonic ultrasonic(3);

// =====================================
// SERVO
// =====================================

Servo monServo;

#define SERVO_PIN 4

// =====================================
// SETUP
// =====================================

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  monServo.attach(SERVO_PIN);
  // position départ
  monServo.write(90);
  Serial.println("MODE TUNNEL");
}
// =====================================
// PILOTAGE MOTEUR
// =====================================

void piloterMoteur(byte adresse, byte direction, byte vitesse)
{
  if (vitesse > 63)
  {
    vitesse = 63;
  }

  byte commande = (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

// =====================================
// DEPLACEMENTS
// =====================================

void avancer()
{
  piloterMoteur(MOTEUR_A, AVANT, 50);
  piloterMoteur(MOTEUR_B, AVANT, 50);
}

// ----------------------------

void gauche()
{
  // moteur gauche stop
  piloterMoteur(MOTEUR_A, ARRET, 0);
  // moteur droit avance
  piloterMoteur(MOTEUR_B, AVANT, 50);
}

// ----------------------------

void droite()
{
  // moteur gauche avance
  piloterMoteur(MOTEUR_A, AVANT, 50);
  // moteur droit stop
  piloterMoteur(MOTEUR_B, ARRET, 0);
}

// ----------------------------

void stopRobot()
{
  piloterMoteur(MOTEUR_A, ARRET, 0);
  piloterMoteur(MOTEUR_B, ARRET, 0);
}

// =====================================
// LOOP
// =====================================

void loop()
{
  // =====================================
  // REGARDER A GAUCHE
  // =====================================

  for(int angle = 90; angle >= 45; angle--)
  {
    monServo.write(angle);
    delay(30);
  }
  delay(200);
  long distanceGauche = ultrasonic.MeasureInCentimeters();

  // =====================================
  // REGARDER A DROITE
  // =====================================

  for(int angle = 45; angle <= 135; angle++)
  {
    monServo.write(angle);
    delay(30);
  }
  delay(200);
  long distanceDroite = ultrasonic.MeasureInCentimeters();

  // =====================================
  // RETOUR CENTRE
  // =====================================

  for(int angle = 135; angle >= 90; angle--)
  {
    monServo.write(angle);
    delay(30);
  }

  // =====================================
  // AFFICHAGE
  // =====================================

  Serial.print("Gauche : ");
  Serial.print(distanceGauche);
  Serial.print(" cm | Droite : ");
  Serial.print(distanceDroite);
  Serial.println(" cm");

  // =====================================
  // LOGIQUE TUNNEL
  // =====================================

  // mur gauche proche
  if (distanceGauche < 20)
  {
    droite();
  }

  // mur droit proche
  else if (distanceDroite < 20)
  {
    gauche();
  }

  // milieu tunnel
  else
  {
    avancer();
  }

  delay(50);
}