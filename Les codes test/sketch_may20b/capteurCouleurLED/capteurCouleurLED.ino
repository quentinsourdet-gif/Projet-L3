#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN    6
#define LED_COUNT  30

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

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

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (c < 10) return;

  float scale = 255.0 / c;
  uint8_t red   = constrain((int)(r * scale), 0, 255);
  uint8_t green = constrain((int)(g * scale), 0, 255);
  uint8_t blue  = constrain((int)(b * scale), 0, 255);

  Serial.print("R:"); Serial.print(red);
  Serial.print(" G:"); Serial.print(green);
  Serial.print(" B:"); Serial.println(blue);

  uint8_t ledR = 0, ledG = 0, ledB = 0;

  if (red > green && red > blue && red > 80) {
    ledR = 255;
    Serial.println("ROUGE");
  } else if (green > red && green > blue && green > 80) {
    ledG = 255;
    Serial.println("VERT");
  } else if (blue > red && blue > green && blue > 80) {
    ledB = 255;
    Serial.println("BLEU");
  } else if (red > 150 && green > 100 && blue < 80) {
    ledR = 255; ledG = 165;
    Serial.println("ORANGE");
  } else if (red > 150 && green > 150 && blue < 100) {
    ledR = 255; ledG = 255;
    Serial.println("JAUNE");
  } else if (red > 150 && blue > 120 && green < 80) {
    ledR = 128; ledB = 128;
    Serial.println("VIOLET");
  } else {
    Serial.println("Couleur inconnue");
    delay(300);
    return;
  }

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
  delay(2000);
}
