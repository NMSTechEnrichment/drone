#include <Servo.h>

Servo servo;

int DIAL_PIN = A0;

void setup() {

  // Starting the Serial Monitor
  Serial.begin(9600);           

  servo.attach(2); //D4

  servo.write(0);

  delay(2000);

}

void loop() {

  int dialVal = analogRead(DIAL_PIN);
  Serial.print("Dial Val: ");
  Serial.println(dialVal);

  int servoPos = map(dialVal, 0, 1023, 0, 180);

  Serial.print("Servo Position: ");
  Serial.println(servoPos);

  servo.write(servoPos);

  delay(100);


}
