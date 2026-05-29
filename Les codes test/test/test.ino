#include <Wire.h>

#define LINE_FOLLOWER 0x20

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("=== TEST CAPTEUR NORMALISE ===");
}

void loop()
{
  Wire.beginTransmission(LINE_FOLLOWER);
  Wire.write(0x02);
  Wire.endTransmission(false);
  Wire.requestFrom(LINE_FOLLOWER, 1);

  if (Wire.available())
  {
    byte val = Wire.read() & 0x0F;

    if (val == 0)
    {
      Serial.println("val = 0  →  PAS DE LIGNE");
    }
    else
    {
      int s4 = (val >> 3) & 1;
      int s3 = (val >> 2) & 1;
      int s2 = (val >> 1) & 1;
      int s1 = (val >> 0) & 1;

      // score brut entre -3 et +3
      int score = (s4 * -3) + (s3 * -1) + (s2 * 1) + (s1 * 3);

      // normalisation entre -512 et +512
      int normalise = map(score, -3, 3, -512, 512);

      Serial.print("val = "); Serial.print(val);
      Serial.print("  score = "); Serial.print(score);
      Serial.print("  normalise = "); Serial.print(normalise);
      Serial.print("  →  ");

      if      (normalise < -170) Serial.println("DROITE");
      else if (normalise >  170) Serial.println("GAUCHE");
      else                       Serial.println("CENTRE");
    }
  }

  delay(200);
}