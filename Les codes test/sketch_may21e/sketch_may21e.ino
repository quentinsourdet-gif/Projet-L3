#include <Wire.h>

#define ADDR_LF     0x20
#define REG_DIGITAL 0x07

uint8_t lireCapteurs() {
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0xFF;
  return Wire.read() & 0x0F;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("=== CALIBRAGE CAPTEUR ===");
  Serial.println("Pose le robot sur chaque surface et note la valeur e:");
}

void loop() {
  uint8_t e = lireCapteurs();
  
  Serial.print("e: ");
  Serial.print(e, BIN);
  Serial.print("  (");
  Serial.print(e, DEC);
  Serial.print(")  → ");

  switch (e) {
    case 0b0000: Serial.println("NOIR TOTAL"); break;
    case 0b1111: Serial.println("BLANC TOTAL"); break;
    case 0b0110: Serial.println("CENTRE"); break;
    case 0b1001: Serial.println("CENTRE"); break;
    case 0b1000: Serial.println("deviation droite"); break;
    case 0b1100: Serial.println("deviation droite forte"); break;
    case 0b1110: Serial.println("deviation droite tres forte"); break;
    case 0b0001: Serial.println("deviation gauche"); break;
    case 0b0011: Serial.println("deviation gauche forte"); break;
    case 0b0111: Serial.println("deviation gauche tres forte"); break;
    default:     Serial.println("autre"); break;
  }

  delay(200);
}