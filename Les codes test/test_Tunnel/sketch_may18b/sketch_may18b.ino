#include <Wire.h>
#include <Servo.h>

// ============================================
// SERVO + ULTRASON
// ============================================

Servo monServo;

#define TRIG_PIN 7
#define ECHO_PIN 6

// ============================================
// MOTEURS
// ============================================

#define MOTEUR_DROIT  0x66
#define MOTEUR_GAUCHE 0x68

#define ARRET   0x00
#define AVANT   0x01
#define ARRIERE 0x02

// ============================================
// FONCTION MOTEUR
// ============================================

void piloterMoteur(byte adresse, byte direction, byte vitesse) {

  if (vitesse > 63) vitesse = 63;

  byte commande = (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

// ============================================
// DEPLACEMENTS
// ============================================

void avancer(int vitesse) {

  piloterMoteur(MOTEUR_DROIT, AVANT, vitesse);
  piloterMoteur(MOTEUR_GAUCHE, AVANT, vitesse);
}

void stopRobot() {

  piloterMoteur(MOTEUR_DROIT, ARRET, 0);
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
}

void tournerGauche(int vitesse) {

  piloterMoteur(MOTEUR_DROIT, AVANT, vitesse);
  piloterMoteur(MOTEUR_GAUCHE, ARRIERE, vitesse);

  delay(500);

  stopRobot();
}

void tournerDroite(int vitesse) {

  piloterMoteur(MOTEUR_DROIT, ARRIERE, vitesse);
  piloterMoteur(MOTEUR_GAUCHE, AVANT, vitesse);

  delay(500);

  stopRobot();
}

// ============================================
// MESURE DISTANCE
// ============================================

long mesurerDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duree = pulseIn(ECHO_PIN, HIGH);

  long distance = duree * 0.034 / 2;

  return distance;
}

// ============================================
// REGARDER GAUCHE
// ============================================

long regarderGauche() {

  monServo.write(150);

  delay(500);

  return mesurerDistance();
}

// ============================================
// REGARDER DROITE
// ============================================

long regarderDroite() {

  monServo.write(30);

  delay(500);

  return mesurerDistance();
}

// ============================================
// SETUP
// ============================================

void setup() {

  Wire.begin();

  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  monServo.attach(4);

  monServo.write(90);

  Serial.println("Robot tunnel autonome pret !");
}

// ============================================
// LOOP
// ============================================

void loop() {

  monServo.write(90);

  long distanceFront = mesurerDistance();

  Serial.print("Distance devant : ");
  Serial.println(distanceFront);

  // ========================================
  // SI LE CHEMIN EST LIBRE
  // ========================================

  if (distanceFront > 25) {

    avancer(40);
  }

  // ========================================
  // OBSTACLE
  // ========================================

  else {

    stopRobot();

    delay(300);

    long distanceGauche = regarderGauche();

    long distanceDroite = regarderDroite();

    Serial.print("Gauche : ");
    Serial.println(distanceGauche);

    Serial.print("Droite : ");
    Serial.println(distanceDroite);

    monServo.write(90);

    // ==========================
    // CHOISIR LE MEILLEUR COTE
    // ==========================

    if (distanceGauche > distanceDroite) {

      tournerGauche(40);
    }
    else {

      tournerDroite(40);
    }
  }

  delay(100);
}