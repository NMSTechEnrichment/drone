#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <PID_v1.h>

#include <MPU6050_light.h>
#include <math.h>

#define FRONT_RIGHT 9
#define FRONT_LEFT 6
#define BACK_RIGHT 3
#define BACK_LEFT 5

// Struct representing the control data coming from the remote. Should not exceed 32 bytes.
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

// Speeds for each motor, 0 t0 255 for 0% to 100% power.
struct MotorSpeed {
  int frontRight;
  int frontLeft;
  int backRight;
  int backLeft;
};

RF24 radio(7, 10); // CE, CSN
const byte address[10] = "ADDRESS01";

MPU6050 mpu(Wire);

ControlData data;

MotorSpeed currentSpeed, targetSpeed;

int pitch = 0;
int roll = 0;
int yaw = 0;

// Timers
unsigned long motorTimer = 0;
unsigned long timer = 0;
float timeStep = 0.01;

// PID 
double desiredRoll, rollInput, rollOutput = 0;
double desiredPitch, pitchInput, pitchOutput = 0; 
double desiredYaw, yawInput, yawOutput = 0; 

// Motor Speeds, target and current
// float frontRightTargetSpeed, frontLeftTargetSpeed, backRightTargetSpeed, backLeftTargetSpeed;
// float frontRightCurrentSpeed, frontLeftCurrentSpeed, backRightCurrentSpeed, backLeftCurrentSpeed;

//Define the aggressive and conservative Tuning Parameters: 
double consKp = 2, consKi = 0.0, consKd = 0.0; //double consKp = 1, consKi = 0.05, consKd = 0.25;
PID pitchPID(&rollInput, &rollOutput, &desiredRoll, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &desiredPitch, consKp, consKi, consKd, DIRECT);
PID yawPID(&yawInput, &yawOutput, &desiredYaw, consKp, consKi, consKd, DIRECT);

void setup() {
  Serial.begin(9600);

  Serial.print("Initializing...");

  Wire.begin();

  // initRadioComms();

  initGyroscope();

  initPID();

  // Set the target speed.
  setSpeed(10);

  Serial.print("Done...");
}

void loop() {
  
  // timer = millis();

  // checkGryoSettings();
  // printAMPUGyroValues();
  readMPUAngles(false);

  // Serial.println("Waiting for data...");
  // if (radio.available()) {
  //   // Serial.println("Data available...");
  //   // char txt[32] = "";
  //   // radio.read(&txt, sizeof(txt));
  //   radio.read(&data, sizeof(data));
  //   printData(data);
  // }

  // pitchPID.Compute();
  // rollPID.Compute();



  if((millis()-motorTimer) > 10){ // print data every 10ms
    
    printMPUValues();
    computePID();
    stabilize(false);
    // runMotors(currentSpeed);
    motorTimer = millis();

  }
  

  // delay(500);
  
  // Wait to full timeStep period
  // delay((timeStep*1000) - (millis() - timer));
  //delay(500);
  

};

void initPID() {
  
  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);  
  yawPID.SetOutputLimits(-20, 20);  
  
  // Set the initial values.
  desiredPitch = 0.0;
  desiredRoll = 0.0;
  desiredYaw = 0.0;

  pitchInput = 0.0;
  rollInput = 0.0;  
  yawInput = 0.0;  

  pitchOutput = 0.0;
  rollOutput = 0.0;  
  yawOutput = 0.0;  


}

void computePID() {
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();

  // if((millis()-timer)>100){ // print data every 10ms
  //   Serial.print("Pitch Diff: ");
  //   Serial.print(pitchOutput);
  //   Serial.print(" Roll Diff: ");
  //   Serial.print(rollOutput);
  //   Serial.println();
  //   timer = millis();
  // }
}

bool initGyroscope() {
  // Try to initialize the adafruit mpu
  if (mpu.begin() != 0) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  delay(250);
  mpu.calcOffsets();
  return true;
}

void initMotors() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FRONT_RIGHT, OUTPUT);
  pinMode(FRONT_LEFT, OUTPUT);
  pinMode(BACK_RIGHT, OUTPUT);
  pinMode(BACK_LEFT, OUTPUT);
}

void setSpeed(float newSpeed) {
  
  currentSpeed.frontRight = newSpeed;
  currentSpeed.frontLeft = newSpeed;
  currentSpeed.backRight = newSpeed;
  currentSpeed.backLeft = newSpeed;

  targetSpeed.frontRight = newSpeed;
  targetSpeed.frontLeft = newSpeed;
  targetSpeed.backRight = newSpeed;
  targetSpeed.backLeft = newSpeed;
}

void runMotors(MotorSpeed toRun) {
  analogWrite(FRONT_RIGHT, toRun.frontRight);  
  analogWrite(FRONT_LEFT, toRun.frontLeft);  
  analogWrite(BACK_RIGHT, toRun.backRight);  
  analogWrite(BACK_LEFT, toRun.backLeft);
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

void readMPUAngles(bool print) {
  mpu.update();
  
  pitchInput = mpu.getAngleX();
  rollInput = mpu.getAngleY();
  yawInput = mpu.getAngleZ(); // todo map to 0 to 255?

 
  if (print) {
    Serial.print(pitchInput);// pitch
    Serial.print(",");
    Serial.print(rollInput);// roll
    Serial.print(",");
    Serial.print(yawInput); //yaw
    Serial.println("");
    // timer = millis();  
  }
}

void printMPUValues() {
  
  Serial.print(pitchInput);// pitch
  Serial.print(",");
  Serial.print(rollInput);// roll
  Serial.print(",");
  Serial.print(yawInput); //yaw
  Serial.println("");  
}

void printPIDOutputValues() {
  
  Serial.print(pitchOutput);// pitch
  Serial.print(",");
  Serial.print(rollOutput);// roll
  Serial.print(",");
  Serial.print(yawOutput); //yaw
  Serial.println("");
   
}


void stabilize(bool log) {

  // Motors front to back control pitch, motors left to right control roll, motors corner to corner control yaw

  // Pitch: Inverse the pitchOutput between front and back motors.
  // Roll: Inverse the rollOutput between left and right motors.
  // Yaw: Inverse the yawOutput between opposing corners
  currentSpeed.frontLeft = targetSpeed.frontLeft +  pitchOutput + rollOutput - yawOutput; //(rollOutput) - (pitchOutput);
  currentSpeed.frontRight = targetSpeed.frontRight + pitchOutput - rollOutput + yawOutput; //(rollOutput) + (pitchOutput);

  currentSpeed.backLeft = targetSpeed.backLeft -  pitchOutput + rollOutput + yawOutput; //(rollOutput) - (pitchOutput);
  currentSpeed.backRight = targetSpeed.backRight - pitchOutput - rollOutput - yawOutput; //(rollOutput) + (pitchOutput);
  

  // if((millis()-timer)>100){ // print data every 10ms
  //   Serial.print(pitchInput);// pitch
  //   Serial.print(", ");
  //   Serial.print(pitchOutput);
  //   Serial.print(", ");
  //   Serial.print(rollInput);// roll
  //   Serial.print(", ");
  //   Serial.print(rollOutput); 
  //   Serial.print(", ");
  //   Serial.print(yawInput);// yaw
  //   Serial.print(", ");
  //   Serial.print(yawOutput); 
    

  //   printMotorSpeed(currentSpeed);
  //   timer = millis();
  // }
}

void printMotorSpeed(MotorSpeed toPrint) {
  Serial.print(" FR: ");
  Serial.print(toPrint.frontRight);
  Serial.print(" FL: ");
  Serial.print(toPrint.frontLeft);
  Serial.print(" BR: ");
  Serial.print(toPrint.backRight);
  Serial.print(" BL: ");
  Serial.print(toPrint.backLeft);
  Serial.println("");
}

void oldStabilize (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff

  actSpeed[0] = (int) currSpeed[0] + (rollDiff / 2) - (pitchDiff / 2); //FL
  actSpeed[1] = (int) currSpeed[1] + (rollDiff / 2) + (pitchDiff / 2); //FR
  actSpeed[2] = (int) currSpeed[2] - (rollDiff / 2) + (pitchDiff / 2); //BR
  actSpeed[3] = (int) currSpeed[3] - (rollDiff / 2) - (pitchDiff / 2); //BL

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0) actSpeed[i] = 0;  
  }

  Serial.print(F("   mot[0]="));
  Serial.print(actSpeed[0]);
  Serial.print(F("   mot[1]="));
  Serial.print(actSpeed[1]);
  Serial.print(F("   mot[2]="));
  Serial.print(actSpeed[2]);
  Serial.print(F("   mot[3]="));
  Serial.println(actSpeed[3]);

}