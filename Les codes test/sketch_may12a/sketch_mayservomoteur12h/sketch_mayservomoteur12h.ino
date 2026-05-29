#include <Servo.h>  // On importe la bibliotheque Servo
                    // elle est deja installee par defaut
                    // elle permet de controler un servo moteur

Servo monServo;     // On cree un objet "monServo"
                    // c'est comme une variable mais pour un servo

void setup() {
  Serial.begin(9600);     // On demarre le moniteur serie
  
  monServo.attach(4);     // On dit que le servo est branche sur D4
                          // si tu changes de port, change juste ce chiffre
  
  Serial.println("Test servo !");  // On affiche un message
}

void loop() {

  // --- POSITION 0 DEGRES ---
  Serial.println("0 degres");
  monServo.write(0);    // On tourne le servo a 0 degres
                        // = position completement a gauche
  delay(1000);          // On attend 1 seconde

  // --- POSITION 90 DEGRES ---
  Serial.println("90 degres");
  monServo.write(90);   // On tourne le servo a 90 degres
                        // = position au milieu
  delay(1000);          // On attend 1 seconde

  // --- POSITION 180 DEGRES ---
  Serial.println("180 degres");
  monServo.write(180);  // On tourne le servo a 180 degres
                        // = position completement a droite
  delay(1000);          // On attend 1 seconde

}