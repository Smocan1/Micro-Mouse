#define STBY_PIN 11  // Standby pin
#define PWMA_PIN 10  // PWM pin for Motor A
#define PWMB_PIN 12
#define BIN1_PIN 8  // Direction control pin 1 for Motor B
#define BIN2_PIN 9  // Direction control pin 2 for Motor B
#define AIN1_PIN 4  // Direction control pin 1 for Motor A
#define AIN2_PIN 5  // Direction control pin 2 for Motor A
int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin3 = 6; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 7; //Encoder Otput 'B' must connected with intreput pin of arduino.
int motorSpeedA = 31;  // Initial motor speed (0-255)
int motorSpeedB = 30;  // Initial motor speed (0-255)
int turnAmount=210;
volatile int lastEncodedA = 0; // Here updated value of encoder store.
volatile int lastEncodedB = 0; // Here updated value of encoder store.
volatile long encoderValueA = 0; // Raw en
volatile long encoderValueB = 0; // Raw en
bool StopMov = false;
void setup() {
  
  pinMode(STBY_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  Serial.begin(9600); //initialize serial comunication

   pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(2, updateEncoderA, CHANGE); 
  attachInterrupt(3, updateEncoderA, CHANGE);
  attachInterrupt(6, updateEncoderB, CHANGE); 
  attachInterrupt(7, updateEncoderB, CHANGE);

}

void loop() {

//for (int i = 0; i <= 500; i++){
  if(encoderValueA<=turnAmount && StopMov == false)
  {
  StopMov = true;
 analogWrite(PWMA_PIN, motorSpeedA);
 analogWrite(PWMB_PIN, motorSpeedB);
 digitalWrite(AIN1_PIN, HIGH); 
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, HIGH); 
  digitalWrite(BIN2_PIN, LOW);
 Serial.print("Forward  ");
 Serial.println(encoderValueA);
 Serial.println(encoderValueB);
 //delay(1000);
  }
  else
  {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
 
  }
//}

//delay(5000);
/*
for (int i = 0; i <= 500; i++){
digitalWrite(MotFwd, HIGH); 
 digitalWrite(MotRev, LOW);
 Serial.print("Reverse  ");
 Serial.println(encoderValue);
}

delay(1000);
*/
} 

void updateEncoderA(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encodedA = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedA << 2) | encodedA; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueA --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueA ++;

  lastEncodedA = encodedA; //store this value for next time
}


void updateEncoderB(){
  int MSB2 = digitalRead(encoderPin3); //MSB = most significant bit
  int LSB2 = digitalRead(encoderPin4); //LSB = least significant bit

  int encodedB = (MSB2 << 1) |LSB2; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encodedB; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueB --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueB ++;

  lastEncodedB = encodedB; //store this value for next time
}