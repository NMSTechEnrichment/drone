#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

float vol = 0;
int input = 0;

int vdividerPin = A7;
int throttlePin =  A3;
int yawPin = A1;
int pitchPin = A2;
int rollPin = A6;
int aux1Pin = 5; // Left switch to the right
int aux2Pin = 3; // Right switch to the left

// Struct representing the control data coming from the remote. Should not exceed 32 bytes.
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};


LiquidCrystal_I2C lcd(0x27, 16, 2);


// const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!
const byte address[10] = "ADDRESS01";// Address needs to match reciever.

RF24 radio(10, 9); // select  CE and CSN  pins

ControlData data;

// Variables to handle LCD update timing
unsigned long lcdUpdateTime = 0;
const unsigned long lcdUpdateInterval = 100; // Update interval in milliseconds

void resetData()
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.

  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
}

void setup()
{
  //Start everything up
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  // radio.setAutoAck(false);
  // radio.setDataRate(RF24_250KBPS);
  // radio.openWritingPipe(address);
  resetData();

  lcd.init();                          // initialize the lcd
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.clear();
  lcd.print("Arduino");
  lcd.setCursor(0, 1);
  lcd.print("Transmitter");
  delay(1200);
  lcd.clear();
  lcd.print("For Arduino");
  lcd.setCursor(0, 1);
  lcd.print("Mini FPV Drone");  
  delay(2200);
  lcd.clear();
  pinMode(vdividerPin, INPUT);
  Serial.begin(9600);               // starting the Serial Monitor

  pinMode(5, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
}

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  unsigned long currentMillis = millis();

  // Update LCD display at regular intervals
  if (currentMillis - lcdUpdateTime >= lcdUpdateInterval) {
    updateLCD();
    lcdUpdateTime = currentMillis;
  }
  // The calibration numbers used here should be measured
  // for your joysticks till they send the correct values.
  data.throttle = mapJoystickValues( analogRead(throttlePin), 13, 524, 1015, true );
  data.yaw      = mapJoystickValues( analogRead(yawPin), 50, 505, 1020, true );
  data.pitch    = mapJoystickValues( analogRead(pitchPin), 12, 544, 1021, true );
  data.roll     = mapJoystickValues( analogRead(rollPin), 34, 522, 1020, true );
  data.AUX1     = digitalRead(aux1Pin);
  data.AUX2     = digitalRead(aux2Pin);

  printData(data);

  // const char txt[] = "Hello World";
  // radio.write(&txt, sizeof(txt));
  radio.write(&data, sizeof(ControlData));
  delay(250);
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

void updateLCD() {
  input = analogRead(vdividerPin);
  vol = (input * 10.0) / 1024.0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voltage:");
  lcd.setCursor(0, 1);
  lcd.print(vol);
  lcd.print("Vx");
}