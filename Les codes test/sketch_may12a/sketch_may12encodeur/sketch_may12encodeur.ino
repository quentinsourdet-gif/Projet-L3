#define SIG1 4

void setup() {
  Serial.begin(9600);
  pinMode(SIG1, INPUT_PULLUP);  // PULLUP important !
  Serial.println("Test encodeur sur D4");
}

void loop() {
  int val = digitalRead(SIG1);
  Serial.println(val);
  delay(100);
}