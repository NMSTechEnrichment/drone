#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MPU 6050 interface using the Adafruit library.
Adafruit_MPU6050 mpu;

float accXError, accYError, accZError, rotXError, rotYError, rotZError;

// Keep track of the time between loops.
long startTime, currentTime;

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

  startTime = millis();

  // TODO Are the values correct?
  calculateGyroError();

}

// Read the sensor.
void loop() {
  
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

  // TODO How do we determine orientation (pitch, roll, yaw)? 
  // What do we know? Time between updates and the rate of change at each loop in radians. 
  // We can then determine how many radians were traversed.

  // The temperature
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  currentTime = millis();
  changeInTime = currentTime - startTime;
  Serial.print("Time since last update: ");
  Serial.println(changeInTime);
  startTime = currentTime;

  Serial.println("");
  delay(500);
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

  delay(1000);

}
