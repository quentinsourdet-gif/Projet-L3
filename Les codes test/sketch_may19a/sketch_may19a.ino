#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN        6
#define LED_COUNT      30
#define PIN_ULTRASON   7
#define DISTANCE_ARRET 10   // cm, à ajuster selon tests

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------------------------------------------------------
//  Ultrason
// -------------------------------------------------------
long mesurerDistance() {
  pinMode(PIN_ULTRASON, OUTPUT);
  digitalWrite(PIN_ULTRASON, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASON, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASON, LOW);

  pinMode(PIN_ULTRASON, INPUT);
  long duree = pulseIn(PIN_ULTRASON, HIGH, 30000);
  return duree / 58;
}

// -------------------------------------------------------
//  Détection couleur + clignotement LEDs
// -------------------------------------------------------
bool detecterEtAfficher() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (c < 10) return false;

  float scale = 255.0 / c;
  uint8_t red   = constrain((int)(r * scale), 0, 255);
  uint8_t green = constrain((int)(g * scale), 0, 255);
  uint8_t blue  = constrain((int)(b * scale), 0, 255);

  Serial.print("R:"); Serial.print(red);
  Serial.print(" G:"); Serial.print(green);
  Serial.print(" B:"); Serial.println(blue);

  uint8_t ledR = 0, ledG = 0, ledB = 0;

  if (red > green && red > blue) {
    ledR = 255;
    Serial.println("-> ROUGE");
  } else if (green > red && green > blue) {
    ledG = 255;
    Serial.println("-> VERT");
  } else if (blue > red && blue > green) {
    ledB = 255;
    Serial.println("-> BLEU");
  } else {
    Serial.println("Indetermine, on reessaie...");
    return false;
  }

  // Clignotement 3 secondes à 2Hz
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < LED_COUNT; j++)
      strip.setPixelColor(j, strip.Color(ledR, ledG, ledB));
    strip.show();
    delay(250);
    strip.clear();
    strip.show();
    delay(250);
  }

  strip.clear();
  strip.show();
  return true;
}

// -------------------------------------------------------
//  Setup
// -------------------------------------------------------
void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.clear();
  strip.show();

  if (!tcs.begin()) {
    Serial.println("Capteur non detecte !");
    while (1);
  }
  Serial.println("Pret !");
}

// -------------------------------------------------------
//  Loop
// -------------------------------------------------------
void loop() {
  long distance = mesurerDistance();
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= DISTANCE_ARRET && distance > 0) {
    // Panneau détecté → lecture couleur + LEDs
    if (detecterEtAfficher()) {
      Serial.println("Couleur detectee -> en attente");
      while (true) { delay(1000); }
    }
  }

  delay(50);
}