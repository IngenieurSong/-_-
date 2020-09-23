#define PIN_LED 7

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_LED, 0);
  delay(1000);
  digitalWrite(PIN_LED, 1);
  delay(200);
  
  for(int i=0; i<5; i++) {
    digitalWrite(PIN_LED, 0);
    delay(200);
    digitalWrite(PIN_LED, 1);
    delay(200);
  }

  while(1) {}
}
