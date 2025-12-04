#include <Servo.h>

Servo servo1;

#define SERVO3 D2 // Digital Pin 4
#define SERVO2 D3 // Digital Pin 0
#define SERVO1 D4 // Digital Pin 2

#define SWITCH_LEFT D5 // Digital Pin 14
#define SWITCH_RIGHT D6 // Digital Pin 12


// The dial is on A0
int DIAL_PIN = A0;

// Setup
void setup() {

  // Starting the Serial Monitor
  Serial.begin(9600);  

  // Initialize the digital pins
  pinMode(SWITCH_LEFT, INPUT_PULLUP);
  pinMode(SWITCH_RIGHT, INPUT_PULLUP);

  // Servo 1
  servo1.attach(SERVO1, 600, 2400, 90); //Attach to D4, calibrate it with the minimum and maximum pwm values for the HS-311 servo.
  servo1.write(0);

  delay(2000);

}

// Loop
void loop() {

  int leftSwitchPosition1 = digitalRead(SWITCH_LEFT);
  int rightSwitchPosition1 = digitalRead(SWITCH_RIGHT);


  int dialVal = analogRead(DIAL_PIN);
  Serial.print("Dial Val: ");
  Serial.println(dialVal);
  
  int servoPos = map(dialVal, 0, 1023, 0, 180);
  Serial.print("Servo Position: ");
  Serial.println(servoPos);

  
  if(leftSwitchPosition1 == 0 && rightSwitchPosition1 == 1)
  {
    servo1.write(servoPos);
  }

  delay(100);

}
