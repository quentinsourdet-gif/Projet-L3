#include <Wire.h>

#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68
#define LINE_FOLLOWER  0x20

#define ARRET   0x00
#define AVANT   0x01
#define ARRIERE 0x02

void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

void avancer() {
  piloterMoteur(MOTEUR_DROIT,  ARRIERE, 40);
  piloterMoteur(MOTEUR_GAUCHE, AVANT,   40);
}

void gauche() {
  piloterMoteur(MOTEUR_DROIT,  ARRIERE, 50);
  piloterMoteur(MOTEUR_GAUCHE, AVANT,   0);
}

void droite() {
  piloterMoteur(MOTEUR_DROIT,  ARRIERE, 0);
  piloterMoteur(MOTEUR_GAUCHE, AVANT,   50);
}

void stopRobot() {
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
}

int lireLigne() {
  // Lire le registre 0x00 = état binaire direct
  Wire.beginTransmission(LINE_FOLLOWER);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(LINE_FOLLOWER, 1);

  if (Wire.available()) {
    byte val = Wire.read();

    // Extraire chaque capteur depuis l'octet
    bool L1 = (val >> 3) & 0x01;
    bool L2 = (val >> 2) & 0x01;
    bool L3 = (val >> 1) & 0x01;
    bool L4 = (val >> 0) & 0x01;

    Serial.print("VAL="); Serial.print(val);
    Serial.print(" L1="); Serial.print(L1);
    Serial.print(" L2="); Serial.print(L2);
    Serial.print(" L3="); Serial.print(L3);
    Serial.print(" L4="); Serial.print(L4);
    Serial.print(" => ");

    // 1 = ligne noire détectée, 0 = pas de ligne
    if      (L2 && L3)           { Serial.println("CENTRE");       return 0;  }
    else if (L1 && L2)           { Serial.println("GAUCHE");       return -1; }
    else if (L3 && L4)           { Serial.println("DROITE");       return 1;  }
    else if (L1 && !L2)          { Serial.println("TRES GAUCHE");  return -2; }
    else if (!L3 && L4)          { Serial.println("TRES DROITE");  return 2;  }
    else if (L2)                 { Serial.println("GAUCHE");       return -1; }
    else if (L3)                 { Serial.println("DROITE");       return 1;  }
    else                         { Serial.println("PAS DE LIGNE"); return 99; }
  }
  return 99;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  stopRobot();
  Serial.println("Robot pret !");
  delay(1000);
}

void loop() {
  int pos = lireLigne();

  switch (pos) {
    case 0:   avancer(); break;
    case -1:  gauche();  break;
    case -2:  gauche();  break;
    case 1:   droite();  break;
    case 2:   droite();  break;
    default:  stopRobot(); break;
  }

  delay(30);
}