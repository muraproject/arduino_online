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
      delay(150);
      digitalWrite(ledPin, LOW);
      keepOn = false;
    }
    else if (command == 'S') {
      digitalWrite(ledPin, HIGH);
      keepOn = true;
    }
    else if (command == '3') {
      digitalWrite(ledPin, HIGH);
      delay(150);
      digitalWrite(ledPin, LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(150);
      digitalWrite(ledPin, LOW);
      delay(1000);
      digitalWrite(ledPin, HIGH);
      delay(150);
      digitalWrite(ledPin, LOW);
      delay(1000);
    }
    else if (command == 'U') {
      digitalWrite(ledPin, HIGH);
      delay(3000);
      digitalWrite(ledPin, LOW);
      
      
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

