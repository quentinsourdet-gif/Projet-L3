#include <Wire.h>

// ======================================================
// ADRESSES I2C
// ======================================================

#define MOTEUR_GAUCHE  0x66
#define MOTEUR_DROIT   0x68

#define LINE_FOLLOWER  0x20

// ======================================================
// REGISTRES
// ======================================================

#define REG_MOTEUR     0x00
#define REG_CAPTEUR    0x02

// ======================================================
// COMMANDES MOTEURS
// ======================================================

#define STOP       0x00
#define AVANT      0x01
#define ARRIERE    0x02

// ======================================================
// VITESSES
// ======================================================

#define VITESSE_BASE 30
#define VITESSE_MAX 63

// ======================================================
// PID
// ======================================================

float Kp = 14.0;
float Ki = 0.0;
float Kd = 5.0;

float erreur = 0;
float ancienneErreur = 0;
float integral = 0;
float derivee = 0;

float correction = 0;

// ======================================================
// SETUP
// ======================================================

void setup()
{
    Wire.begin();

    Serial.begin(9600);

    Serial.println("ROBOT SUIVEUR PID");
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
// AVANCER PID
// ======================================================

void avancerPID(int vitesseGauche, int vitesseDroite)
{
    // SENS CORRIGE POUR TON ROBOT

    moteur(MOTEUR_GAUCHE, ARRIERE, vitesseGauche);

    moteur(MOTEUR_DROIT, AVANT, vitesseDroite);
}

// ======================================================
// STOP ROBOT
// ======================================================

void stopRobot()
{
    moteur(MOTEUR_GAUCHE, STOP, 0);

    moteur(MOTEUR_DROIT, STOP, 0);
}

// ======================================================
// LECTURE CAPTEUR
// ======================================================

int lireCapteur()
{
    Wire.beginTransmission(LINE_FOLLOWER);

    Wire.write(REG_CAPTEUR);

    Wire.endTransmission();

    Wire.requestFrom(LINE_FOLLOWER, 1);

    if(Wire.available())
    {
        return Wire.read();
    }

    return 255;
}

// ======================================================
// FILTRE MOYENNE
// ======================================================

int lireCapteurFiltre()
{
    int moyenne =
    (
        lireCapteur()
      + lireCapteur()
      + lireCapteur()
      + lireCapteur()
    ) / 4;

    return moyenne;
}

// ======================================================
// CALCUL ERREUR
// ======================================================

float calculErreur(int valeur)
{
    // ==========================================
    // LIGNE AU CENTRE
    // ==========================================

    if(valeur >= 12 && valeur <= 30)
    {
        return 0;
    }

    // ==========================================
    // LIGNE A GAUCHE
    // ==========================================

    else if(valeur > 30 && valeur <= 60)
    {
        return -3;
    }

    // ==========================================
    // LIGNE A DROITE
    // ==========================================

    else if(valeur > 60 && valeur <= 90)
    {
        return 3;
    }

    // ==========================================
    // AUTRES VALEURS
    // ==========================================

    return 0;
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
    // ==========================================
    // LECTURE CAPTEUR FILTRE
    // ==========================================

    int valeur = lireCapteurFiltre();

    // ==========================================
    // AFFICHAGE VALEUR
    // ==========================================

    Serial.print("Valeur : ");

    Serial.println(valeur);

    // ==========================================
    // CALCUL ERREUR
    // ==========================================

    erreur = calculErreur(valeur);

    // ==========================================
    // PID
    // ==========================================

    integral += erreur;

    derivee = erreur - ancienneErreur;

    correction =
        (Kp * erreur)
      + (Ki * integral)
      + (Kd * derivee);

    ancienneErreur = erreur;

    // ==========================================
    // CALCUL VITESSES
    // ==========================================

    int vitesseGauche =
        VITESSE_BASE + correction;

    int vitesseDroite =
        VITESSE_BASE - correction;

    // ==========================================
    // LIMITES
    // ==========================================

    vitesseGauche =
        constrain(vitesseGauche, 20, VITESSE_MAX);

    vitesseDroite =
        constrain(vitesseDroite, 20, VITESSE_MAX);

    // ==========================================
    // LIGNE PERDUE
    // ==========================================

    if(valeur > 95)
    {
        stopRobot();
    }

    // ==========================================
    // DETECTION FOND MARRON
    // ==========================================

    else if(valeur >= 45 && valeur <= 75)
    {
        delay(150);

        int verification = lireCapteurFiltre();

        if(verification >= 45 && verification <= 75)
        {
            stopRobot();
        }
        else
        {
            avancerPID(vitesseGauche, vitesseDroite);
        }
    }

    // ==========================================
    // SUIVI NORMAL
    // ==========================================

    else
    {
        avancerPID(vitesseGauche, vitesseDroite);
    }

    delay(5);
}