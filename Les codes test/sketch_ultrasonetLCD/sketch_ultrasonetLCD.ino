
#include <Wire.h>
#include "rgb_lcd.h"

// ---- Configuration de l'ultrason ----
// Le capteur Ultrasonic Ranger V2.0 Grove utilise UNE seule broche (SIG)
#define ULTRASON_PIN  7   // Branché sur D7 du Grove Base Shield

// ---- Objet LCD ----
rgb_lcd lcd;

void setup() {
  Serial.begin(9600);
  
  // Initialisation de l'écran LCD 16x2
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 255);   // rétroéclairage blanc (modifiable)
  
  lcd.setCursor(0, 0);
  lcd.print("Robot Ultimate");
  lcd.setCursor(0, 1);
  lcd.print("Initialisation");
  delay(1500);
  lcd.clear();
}

void loop() {
  long distance_cm = mesurer_distance();
  
  Serial.print("Distance : ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  
  afficher_distance_lcd(distance_cm);
  
  delay(500);  // rafraîchissement toutes les 0.5s
}

// =====================================================
// FONCTION : mesurer_distance()
// Retourne la distance en cm mesurée par l'ultrason Grove V2.0
// =====================================================
long mesurer_distance() {
  long duree, distance;
  
  // Le capteur Grove utilise la même broche pour Trigger et Echo
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_PIN, LOW);
  
  // Passage en lecture pour récupérer l'écho
  pinMode(ULTRASON_PIN, INPUT);
  duree = pulseIn(ULTRASON_PIN, HIGH, 30000); // timeout 30ms (~5m max)
  
  // Conversion en cm : vitesse du son = 340 m/s = 29 µs/cm aller-retour
  distance = duree / 58;
  
  // Sécurité : si hors plage, retourne -1
  if (duree == 0 || distance > 400) {
    return -1;
  }
  
  return distance;
}

// =====================================================
// FONCTION : afficher_distance_lcd(distance)
// Affiche la distance mesurée sur l'écran LCD 16x2
// =====================================================
void afficher_distance_lcd(long distance) {
  lcd.clear();
  
  // Ligne 1 : titre
  lcd.setCursor(0, 0);
  lcd.print("Distance panier:");
  
  // Ligne 2 : valeur
  lcd.setCursor(0, 1);
  if (distance == -1) {
    lcd.print("Hors portee");
  } else {
    lcd.print(distance);
    lcd.print(" cm");
    
    // Conversion en mètres pour info supplémentaire
    float distance_m = distance / 100.0;
    lcd.print(" (");
    lcd.print(distance_m, 2);
    lcd.print("m)");
  }
}