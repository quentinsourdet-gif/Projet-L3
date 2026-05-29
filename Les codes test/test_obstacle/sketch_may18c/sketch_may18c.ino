#include <Wire.h>
#include <Servo.h>

// ======================================================
// CAPTEUR LIGNE
// ======================================================
#define ADDR_LF      0x20
#define REG_DIGITAL  0x07

// ======================================================
// MOTEURS
// ======================================================
#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68

// 🔥 SENS MOTEURS CORRIGES
#define AVANT_DROIT    0x01
#define ARRIERE_DROIT  0x02

#define AVANT_GAUCHE   0x02
#define ARRIERE_GAUCHE 0x01

#define ARRET          0x00

// ======================================================
// SERVO + ULTRASON
// ======================================================
#define SERVO_PIN    3
#define ULTRASON_PIN 7

#define SERVO_CENTRE 90
#define SERVO_GAUCHE 0
#define SERVO_DROITE 180

Servo monServo;

// ======================================================
// PARAMETRES
// ======================================================
#define DIST_ARRET 25

#define VITESSE_BASE 28
#define VITESSE_MAX  40
#define VITESSE_MIN  15

// ======================================================
// SETUP
// ======================================================
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);

  delay(1000);

  Serial.println("=== ROBOT PRET ===");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
  long distance = mesurerDistanceFiltre();

  if (distance > 0 && distance < DIST_ARRET)
  {
    stopRobot();
    delay(300);

    eviterParGauche();

    return;
  }

  suivreLigneSimple();
}

// ======================================================
// ULTRASON
// ======================================================
long mesurerDistance()
{
  pinMode(ULTRASON_PIN, OUTPUT);

  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(5);

  digitalWrite(ULTRASON_PIN, LOW);

  pinMode(ULTRASON_PIN, INPUT);

  long duree = pulseIn(ULTRASON_PIN, HIGH, 20000);

  if (duree == 0)
    return 999;

  return duree / 29 / 2;
}

long mesurerDistanceFiltre()
{
  long somme = 0;
  int compteur = 0;

  for (int i = 0; i < 3; i++)
  {
    long d = mesurerDistance();

    if (d > 2 && d < 200)
    {
      somme += d;
      compteur++;
    }

    delay(10);
  }

  if (compteur == 0)
    return 999;

  return somme / compteur;
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

  if (!Wire.available())
    return 0xFF;

  return Wire.read() & 0x0F;
}

// ======================================================
// SUIVI DE LIGNE
// ======================================================
void suivreLigneSimple()
{
  uint8_t e = lireCapteurs();

  Serial.println(e, BIN);

  // ligne perdue
  if (e == 0b1111)
  {
    avancer(20);
    return;
  }

  // ligne a gauche
  if (e == 0b0011 || e == 0b0111)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 8);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 30);
    return;
  }

  // ligne a droite
  if (e == 0b1100 || e == 0b1110)
  {
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 30);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 8);
    return;
  }

  // ligne au centre
  avancer(25);
}

// ======================================================
// EVITEMENT OBSTACLE
// ======================================================
void eviterParGauche()
{
  Serial.println("=== EVITEMENT ===");

  stopRobot();
  delay(200);

  // =========================================
  // ROTATION GAUCHE
  // =========================================
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 28);
  piloterMoteur(MOTEUR_DROIT, ARRET, 0);

  delay(500);

  stopRobot();
  delay(150);

  // =========================================
  // AVANCER
  // =========================================
  avancer(24);

  delay(2000);

  stopRobot();
  delay(150);

  // =========================================
  // ROTATION RETOUR
  // =========================================
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT, AVANT_DROIT, 28);

  delay(1200);

  stopRobot();
  delay(150);

  // =========================================
  // RECHERCHE DE LIGNE
  // =========================================
  monServo.write(SERVO_DROITE);

  delay(300);

  unsigned long t0 = millis();

  while (millis() - t0 < 4000)
  {
    uint8_t ligne = lireCapteurs();

    Serial.print("Recherche ligne : ");
    Serial.println(ligne, BIN);

    // balayage plus fort
    piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, 10);
    piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, 32);

    // vraie detection de ligne
    if (ligne == 0b0011 ||
        ligne == 0b0111 ||
        ligne == 0b1100 ||
        ligne == 0b1110)
    {
      stopRobot();

      delay(200);

      // recentrage
      avancer(20);

      delay(250);

      stopRobot();

      break;
    }

    delay(40);
  }

  monServo.write(SERVO_CENTRE);

  delay(300);

  Serial.println("=== FIN EVITEMENT ===");
}

// ======================================================
// MOTEURS
// ======================================================
void avancer(int vitesse)
{
  piloterMoteur(MOTEUR_GAUCHE, AVANT_GAUCHE, vitesse);
  piloterMoteur(MOTEUR_DROIT,  AVANT_DROIT, vitesse);
}

void stopRobot()
{
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
}

void piloterMoteur(byte adresse, byte direction, byte vitesse)
{
  vitesse = constrain(vitesse, 0, 63);

  byte commande = (vitesse << 2) | direction;

  Wire.beginTransmission(adresse);

  Wire.write(0x00);
  Wire.write(commande);

  Wire.endTransmission();
}