#include <Wire.h>

#define MOTEUR_DROIT   0x66
#define MOTEUR_GAUCHE  0x68
#define LINE_FOLLOWER  0x20

#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02

#define VITESSE  20

void piloterMoteur(byte adresse, byte direction, byte vitesse)
{
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

void avancer()
{
  piloterMoteur(MOTEUR_DROIT,  AVANT,   VITESSE);
  piloterMoteur(MOTEUR_GAUCHE, ARRIERE, VITESSE);
}

void gauche()
{
  piloterMoteur(MOTEUR_DROIT,  AVANT,   VITESSE);
  piloterMoteur(MOTEUR_GAUCHE, ARRET,   0);
}

void droite()
{
  piloterMoteur(MOTEUR_DROIT,  ARRET,   0);
  piloterMoteur(MOTEUR_GAUCHE, ARRIERE, VITESSE);
}

void stopRobot()
{
  piloterMoteur(MOTEUR_DROIT,  ARRET, 0);
  piloterMoteur(MOTEUR_GAUCHE, ARRET, 0);
}

int lireLigne()
{
  Wire.beginTransmission(LINE_FOLLOWER);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(LINE_FOLLOWER, 1);

  if (!Wire.available()) return 99;

  byte val = Wire.read() & 0x0F;

  if (val == 0) return 99;

  int s4 = (val >> 3) & 1;
  int s3 = (val >> 2) & 1;
  int s2 = (val >> 1) & 1;
  int s1 = (val >> 0) & 1;

  int score = (s4 * -3) + (s3 * -1) + (s2 * 1) + (s1 * 3);

  if      (score < -1) return 1;
  else if (score >  1) return -1;
  else                 return 0;
}

int dernierPos = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  stopRobot();
  delay(1000);
}

void loop()
{
  int pos = lireLigne();

  if (pos != 99) dernierPos = pos;

  switch (pos)
  {
    case  0:  avancer(); break;
    case  1:  droite();  break;
    case -1:  gauche();  break;
    case 99:
      if      (dernierPos ==  1) droite();
      else if (dernierPos == -1) gauche();
      else                       stopRobot();
      break;
  }

  delay(50);
}