#include <Wire.h>

#define LINE_FOLLOWER_ADDRESS 0x20  // Adresse trouvée par le scanner

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("=== Test Me RGB Line Follower ===");
  delay(500);
}

void loop() {
  Wire.beginTransmission(LINE_FOLLOWER_ADDRESS);
  Wire.write(0x01);  // Registre des capteurs
  Wire.endTransmission(false);

  Wire.requestFrom(LINE_FOLLOWER_ADDRESS, 4);

  if (Wire.available() == 4) {
    byte s1 = Wire.read();
    byte s2 = Wire.read();
    byte s3 = Wire.read();
    byte s4 = Wire.read();

    Serial.print("S1="); Serial.print(s1);
    Serial.print(" S2="); Serial.print(s2);
    Serial.print(" S3="); Serial.print(s3);
    Serial.print(" S4="); Serial.print(s4);
    Serial.print("  =>  ");

    if (s2 == 0 && s3 == 0)       Serial.println("CENTRE ✓");
    else if (s1 == 0 && s2 == 0)  Serial.println("GAUCHE <--");
    else if (s3 == 0 && s4 == 0)  Serial.println("DROITE -->");
    else if (s1 == 0)              Serial.println("TRES GAUCHE <<");
    else if (s4 == 0)              Serial.println("TRES DROITE >>");
    else                           Serial.println("PAS DE LIGNE");

  } else {
    Serial.println("Erreur lecture");
  }

  delay(100);
}