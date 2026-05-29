#include <Wire.h>

#define ADDR_D0  0x68
#define ADDR_CC  0x66

// Protocole Seeed Mini I2C Motor Driver
// Trame : [addr_registre] [vitesse_M1] [vitesse_M2]
// Vitesse : 0-127 marche avant, 128-255 marche arriere (256-valeur)

void setMotors(uint8_t addr, int speedM1, int speedM2) {
  uint8_t s1, s2;
  
  // Encodage moteur 1
  if (speedM1 >= 0) s1 = constrain(speedM1, 0, 127);
  else              s1 = constrain(256 + speedM1, 128, 255);
  
  // Encodage moteur 2
  if (speedM2 >= 0) s2 = constrain(speedM2, 0, 127);
  else              s2 = constrain(256 + speedM2, 128, 255);

  Wire.beginTransmission(addr);
  Wire.write(0x01);  // registre de commande
  Wire.write(s1);
  Wire.write(s2);
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Test moteurs OK");
  delay(1000);
}

void loop() {
  Serial.println("Avance");
  setMotors(ADDR_D0, 80, 80);
  setMotors(ADDR_CC, 80, 80);
  delay(2000);

  Serial.println("Stop");
  setMotors(ADDR_D0, 0, 0);
  setMotors(ADDR_CC, 0, 0);
  delay(1000);

  Serial.println("Recule");
  setMotors(ADDR_D0, -80, -80);
  setMotors(ADDR_CC, -80, -80);
  delay(2000);

  Serial.println("Stop");
  setMotors(ADDR_D0, 0, 0);
  setMotors(ADDR_CC, 0, 0);
  delay(1000);
}