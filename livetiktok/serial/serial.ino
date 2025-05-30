const int ledPin = 4;  // Ganti dengan pin sesuai kebutuhan
char command;
bool keepOn = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    command = Serial.read();

    if (command == 'F') {
      digitalWrite(ledPin, HIGH);
      delay(1000);
      digitalWrite(ledPin, LOW);
      keepOn = false;
    }
    else if (command == 'S') {
      digitalWrite(ledPin, HIGH);
      keepOn = true;
    }
    else if (command == 'X') {
      digitalWrite(ledPin, LOW);
      keepOn = false;
    }
  }

  if (keepOn) {
    digitalWrite(ledPin, HIGH);
  }
}
