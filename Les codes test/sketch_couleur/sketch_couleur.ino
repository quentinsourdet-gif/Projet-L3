#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>

// ---- Configuration du ruban LED ----
#define LED_PIN 6
#define NUM_LEDS 30
#define BRIGHTNESS 80

// ---- Coefficients de correction ----
#define COEF_R 1.20
#define COEF_G 1.05
#define COEF_B 1.05
#define SEUIL_CLEAR_MIN 5

// ---- Objets globaux ----
Adafruit_NeoPixel ruban(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS, TCS34725_GAIN_60X);

// ---- Prototypes des fonctions ----
String detecter_couleur();
void afficher_couleur_led(String couleur);

// =====================================================
void setup() {
Serial.begin(9600);

if (colorSensor.begin()) {
Serial.println("Capteur TCS34725 OK !");
} else {
Serial.println("ERREUR capteur !");
while (1);
}

ruban.begin();
ruban.setBrightness(BRIGHTNESS);
ruban.clear();
ruban.show();

Serial.println("Pret pour detection");
delay(1000);
}

// =====================================================
void loop() {
String couleur = detecter_couleur();

Serial.print(">>> Couleur : ");
Serial.println(couleur);
Serial.println("------------------------");

if (couleur != "inconnue") {
afficher_couleur_led(couleur);
}

delay(1500);
}

// =====================================================
// FONCTION : detecter_couleur()
// =====================================================
String detecter_couleur() {
uint16_t red, green, blue, clear;

colorSensor.getRawData(&red, &green, &blue, &clear);

Serial.print("BRUT R="); Serial.print(red);
Serial.print(" G="); Serial.print(green);
Serial.print(" B="); Serial.print(blue);
Serial.print(" C="); Serial.println(clear);

if (clear < SEUIL_CLEAR_MIN) {
Serial.println("-> Trop sombre");
return "inconnue";
}

float r_norm = (float)red / (float)clear;
float g_norm = (float)green / (float)clear;
float b_norm = (float)blue / (float)clear;

float r = r_norm * COEF_R;
float g = g_norm * COEF_G;
float b = b_norm * COEF_B;

Serial.print("CORR R="); Serial.print(r, 3);
Serial.print(" G="); Serial.print(g, 3);
Serial.print(" B="); Serial.println(b, 3);

float ratio_rg = r / g;
float ratio_rb = r / b;
float ratio_gr = g / r;
float ratio_gb = g / b;
float ratio_br = b / r;
float ratio_bg = b / g;

Serial.print("RATIOS R/G="); Serial.print(ratio_rg, 2);
Serial.print(" R/B="); Serial.print(ratio_rb, 2);
Serial.print(" G/B="); Serial.print(ratio_gb, 2);
Serial.print(" B/G="); Serial.println(ratio_bg, 2);

// ROUGE
if (ratio_rg > 1.15 && ratio_rb > 1.15) {
return "rouge";
}

// VERT
if (ratio_gr > 1.05 && ratio_gb > 1.05) {
return "vert";
}

// BLEU
if (ratio_br > 1.05 && ratio_bg > 1.05) {
return "bleu";
}

return "inconnue";
}

// =====================================================
// FONCTION : afficher_couleur_led()
// =====================================================
void afficher_couleur_led(String couleur) {
uint32_t couleurLED;

if (couleur == "rouge") {
couleurLED = ruban.Color(255, 0, 0);
}
else if (couleur == "vert") {
couleurLED = ruban.Color(0, 255, 0);
}
else if (couleur == "bleu") {
couleurLED = ruban.Color(0, 0, 255);
}
else {
couleurLED = ruban.Color(0, 0, 0);
}

for (int i = 0; i < 3; i++) {
for (int j = 0; j < NUM_LEDS; j++) {
ruban.setPixelColor(j, couleurLED);
}
ruban.show();
delay(500);

ruban.clear();
ruban.show();
delay(500);
}
}

