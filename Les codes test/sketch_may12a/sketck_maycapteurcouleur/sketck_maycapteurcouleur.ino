#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>

// Ruban LED
#define LED_PIN     3   // Port D3
#define LED_COUNT   30  // 30 LEDs sur le ruban

// Capteur couleur
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  Serial.println("=== Capteur Couleur + Ruban LED ===");

  // Init ruban
  strip.begin();
  strip.show(); // Eteint toutes les LEDs au démarrage
  strip.setBrightness(50); // Luminosité 50/255

  // Init capteur couleur
  if (tcs.begin()) {
    Serial.println("Capteur couleur OK !");
  } else {
    Serial.println("ERREUR capteur couleur !");
    while (1);
  }
}

// Fonction pour faire clignoter le ruban pendant 3 secondes
void clignoter(uint8_t r, uint8_t g, uint8_t b) {
  Serial.println("Clignotement 3 secondes...");
  
  for (int i = 0; i < 6; i++) {  // 6 fois = 3 secondes à 0.5Hz
    // Allume
    for (int j = 0; j < LED_COUNT; j++) {
      strip.setPixelColor(j, strip.Color(r, g, b));
    }
    strip.show();
    delay(250);

    // Eteint
    strip.clear();
    strip.show();
    delay(250);
  }
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  Serial.print("R="); Serial.print(r);
  Serial.print(" G="); Serial.print(g);
  Serial.print(" B="); Serial.print(b);
  Serial.print(" C="); Serial.print(c);
  Serial.print("  =>  ");

  if (c < 50) {
    Serial.println("TROP SOMBRE");
    strip.clear();
    strip.show();
  }
  else if (r > g && r > b) {
    Serial.println("ROUGE - Clignotement !");
    piloterMoteur(MOTEUR_DROIT,ARRET,0);
    piloterMoteur(MOTEUR_GAUCHE,ARRET,0);
    clignoter(255, 0, 0);  // Rouge
    moveMotors();
  }
  else if (g > r && g > b) {
    Serial.println("VERT - Clignotement !");
    piloterMoteur(MOTEUR_DROIT,ARRET,0);
    piloterMoteur(MOTEUR_GAUCHE,ARRET,0);
    clignoter(0, 255, 0);  // Vert
    moveMotors();
  }
  else if (b > r && b > g) {
    Serial.println("BLEU - Clignotement !");
    piloterMoteur(MOTEUR_DROIT,ARRET,0);
    piloterMoteur(MOTEUR_GAUCHE,ARRET,0);
    clignoter(0, 0, 255);  // Bleu
    moveMotors();
  }
  else if (r > b && g > b) {
    Serial.println("JAUNE - Clignotement !");
    piloterMoteur(MOTEUR_DROIT,ARRET,0);
    piloterMoteur(MOTEUR_GAUCHE,ARRET,0);
    clignoter(255, 255, 0); // Jaune
    moveMotors();
  }
  else {
    Serial.println("INCONNU");
    strip.clear();
    strip.show();
  }

  delay(300);
}