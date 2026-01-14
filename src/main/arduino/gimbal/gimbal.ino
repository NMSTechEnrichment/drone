#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

#define SERVO1 D6 
#define SERVO2 D7 
#define SERVO3 D8 

#define INTERRUPT_PIN D3 // The interrupt pin.

Servo pitchServo;
Servo rollServo;
Servo yawServo;

// MPU 6050 interface using the Adafruit library.
Adafruit_MPU6050 mpu;
volatile bool dataReady = false; 

float accXError, accYError, accZError, rotXError, rotYError, rotZError;

// Keep track of the time between loops.
long startTime, currentTime;

int currentPitch, currentRoll, currentYaw;

// Setup
void setup(void) {

  Serial.begin(115200);
  Serial.println("MPU6050 test!");

  // Try to initialize.
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set the sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("");  
  delay(100);

  // TODO The gyro can tell the board when it changes.
  // pinMode(INTERRUPT_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), gryoDataReady, RISING);
  

  startTime = millis();

  // Are the values correct?
  calculateGyroError();

  // Setup the servos

  // Pitch Servo 1
  pitchServo.attach(SERVO1, 900, 2100);
  pitchServo.write(90);

  // Roll Servo 2
  rollServo.attach(SERVO2, 900, 2100);
  rollServo.write(90);

  // Yaw Servo 3
  yawServo.attach(SERVO3, 900, 2100);
  yawServo.write(90);

  dataReady = true;

}

// Triggers when data is ready.
void gryoDataReady(){
  Serial.print("Stamp(ms): ");
  Serial.println(millis());
  dataReady = true;
}

// Read the sensor.
void loop() {
  
  // TODO can we update only when the gyro moves?
  // if(!dataReady) {
  //   return;
  // }

  long changeInTime = 0;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Acceleration values.
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x - accXError);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y - accYError);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z - accZError);
  Serial.println(" m/s^2");

  // Rotation values.
  Serial.print("Rotation Rate X: ");
  Serial.print(g.gyro.x - rotXError);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y - rotYError);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z - rotZError);
  Serial.println(" rad/s");

  // The temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  currentTime = millis();
  changeInTime = currentTime - startTime;
  Serial.print("Time since last update: ");
  Serial.println(changeInTime);
  startTime = currentTime;

  currentPitch = currentPitch + (calcChangeInDegrees(g.gyro.x - rotXError, changeInTime) * -1.0); // The pitch servo is inverted
  currentRoll = currentRoll + calcChangeInDegrees(g.gyro.y - rotYError, changeInTime);
  currentYaw = currentYaw + calcChangeInDegrees(g.gyro.z - rotZError, changeInTime);

  Serial.print("Orientation X: ");
  Serial.print(currentPitch);
  Serial.print(", Y: ");
  Serial.print(currentRoll);
  Serial.print(", Z: ");
  Serial.print(currentYaw);
  Serial.println(" degrees");

  // Map the current values in range of -90 to +90 to 0 to 180 for the servros
  Serial.print("Servo Orientation X: ");
  Serial.print(map(currentPitch, -90, 90, 180, 0));
  Serial.print(", Y: ");
  Serial.print(map(currentRoll, -90, 90, 0, 180));
  Serial.print(", Z: ");
  Serial.print(map(currentYaw, -90, 90, 0, 180));
  Serial.println(" degrees");
  Serial.println("");

  pitchServo.write(map(currentPitch, -90, 90, 0, 180));
  rollServo.write(map(currentRoll, -90, 90, 0, 180));
  yawServo.write(map(currentYaw, -90, 90, 0, 180));

  delay(150);
  // dataReady = false;
}

// Calculate the change in degrees based on rate and time.
float calcChangeInDegrees(float correctedValue, long changeInTime) {
  float changeInRadians = correctedValue * (changeInTime / 1000.0);
  return changeInRadians * 180 / M_PI;
  
}

// Calculate the errors. 
void calculateGyroError() {

  Serial.println("Calculating Errors");
  int errorCount = 500;
  sensors_event_t a, g, temp;

  // Read it 200 times, accumulating error each time.
  for (int i = 0; i <= errorCount; i++) {

    mpu.getEvent(&a, &g, &temp);

    accXError = accXError + a.acceleration.x;
    accYError = accYError + a.acceleration.y;
    accZError = accZError + (a.acceleration.z - 9.8 ); // Gravity is always pulling down at 9.8m/s^2.

    rotXError = rotXError + g.gyro.x;
    rotYError = rotYError + g.gyro.y;
    rotZError = rotZError + g.gyro.z; 
  }

  // Invert the errors, we want to add or subtrack them from the actual values.
  accXError = accXError / errorCount;
  accYError = accYError / errorCount;
  accZError = accZError / errorCount;

  rotXError = rotXError / errorCount;
  rotYError = rotYError / errorCount;
  rotZError = rotZError / errorCount;

  // Print out the values.
  Serial.print("Acceleration Errors X:");
  Serial.print(accXError);
  Serial.print(", Y: ");
  Serial.print(accYError);
  Serial.print(", Z: ");
  Serial.println(accZError);

  Serial.print("Rotation Errors X:");
  Serial.print(rotXError);
  Serial.print(", Y: ");
  Serial.print(rotYError);
  Serial.print(", Z: ");
  Serial.println(rotZError);
  Serial.println();

  delay(250);

}
