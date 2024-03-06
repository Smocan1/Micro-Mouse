// Define pins
#define STBY_PIN 9  // Standby pin
#define PWMA_PIN 5  // PWM pin for Motor A
#define AIN1_PIN 4  // Direction control pin 1 for Motor A
#define AIN2_PIN 2  // Direction control pin 2 for Motor A

// Motor speed variables
int motorSpeed = 255;  // Initial motor speed (0-255)

void setup() {
  // Initialize pins
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);

  // Set initial motor direction
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);

  // Enable motor driver
  digitalWrite(STBY_PIN, HIGH);
}

void loop() {
  // Set motor speed using PWM
  analogWrite(PWMA_PIN, motorSpeed);

  // Delay for a short duration (adjust as needed)
  delay(1000);

  // Change motor direction (toggle AIN1 and AIN2 pins)
  //digitalWrite(AIN1_PIN, !digitalRead(AIN1_PIN));
  //digitalWrite(AIN2_PIN, !digitalRead(AIN2_PIN));
}
