
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>


#define FRONT_RIGHT 9
#define FRONT_LEFT 6
#define BACK_RIGHT 3
#define BACK_LEFT 5
#define BUZZER 8


// Struct representing the control data coming from the remote. Should not exceed 32 bytes.
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

// The radio reciever.
RF24 radio(7, 10); // CE, CSN

// Data coming from the controller.
ControlData data;

const byte address[10] = "ADDRESS01"; // Has to match the address of the transmitter.



void setup() {
  Serial.begin(9600);
  Serial.print("Initializing...");


  pinMode(FRONT_RIGHT, OUTPUT);
  pinMode(FRONT_LEFT, OUTPUT);
  pinMode(BACK_RIGHT, OUTPUT);
  pinMode(BACK_LEFT, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Wire.begin();
  initRadioComms();

  Serial.print("Done...");

}


void loop() {
  
  Serial.println("Waiting for data...");

  if (radio.available()) {
    radio.read(&data, sizeof(data));
    printData(data);

    
    // Turn the LED on if AUX1 is set to 1, turn it off otherwise.
    if(data.AUX1 == 1) {
      Serial.println("Turning ON motors.");
      tone(BUZZER, 85);
      runAllMotors(0.1);
    } else {
      Serial.println("Turning OFF motors.");
      noTone(BUZZER);
      runAllMotors(0.0);
    }

  }

  delay(250);
}



void runAllMotors(double percentPower) {

  int pwmPower = 255 * percentPower;
  
  Serial.print("Setting motor power to ");
  Serial.println(pwmPower);

  analogWrite(FRONT_RIGHT, pwmPower);  
  analogWrite(FRONT_LEFT, pwmPower);  
  analogWrite(BACK_RIGHT, pwmPower);  
  analogWrite(BACK_LEFT, pwmPower);  
}


void initRadioComms() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}


void printData(ControlData toPrint) {
    Serial.print("Throttle: ");
    Serial.print(toPrint.throttle);
    Serial.print(" Yaw: ");
    Serial.print(toPrint.yaw);
    Serial.print(" Pitch: ");
    Serial.print(toPrint.pitch);
    Serial.print(" Roll: ");
    Serial.print(toPrint.roll);
    Serial.print(" AUX1: ");
    Serial.print(toPrint.AUX1);
    Serial.print(" AUX2: ");
    Serial.print(toPrint.AUX2);
    Serial.println("");

}