#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Scanner I2C");
}
void loop() {
  byte error;
  byte address;
  int nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Capteur trouvé à l'adresse : 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) {
    Serial.println("Aucun périphérique I2C trouvé");
  }
  delay(3000);
}