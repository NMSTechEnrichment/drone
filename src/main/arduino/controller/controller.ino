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
int LEFT_SWITCH_1 = 4; // Left switch to the left
int LEFT_SWITCH_2 = 5; // Left switch to the right
int RIGHT_SWITCH_1 = 3; // Right switch to the left
int RIGHT_SWITCH_2 = 2; // Right switch to the right
int DIAL_PIN = A0;

// Struct representing the control data coming from the remote. Should not exceed 32 bytes.
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte dial;
  byte leftSwitchPosition1;
  byte leftSwitchPosition2;
  byte rightSwitchPosition1;
  byte rightSwitchPosition2;
};


LiquidCrystal_I2C lcd(0x27, 16, 2);


// const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver!!!
// todo Change this based on right switch position.
const byte address[6] = "200000";// Address needs to match reciever.

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
  data.dial = 0;
  data.leftSwitchPosition1 = 0;
  data.leftSwitchPosition2 = 0;
  data.rightSwitchPosition1 = 0;
  data.rightSwitchPosition2 = 0;
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

  pinMode(LEFT_SWITCH_1, INPUT_PULLUP);
  pinMode(LEFT_SWITCH_2, INPUT_PULLUP);
  pinMode(RIGHT_SWITCH_1, INPUT_PULLUP);
  pinMode(RIGHT_SWITCH_2, INPUT_PULLUP);

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
  data.dial     = map(analogRead(DIAL_PIN), 0, 1023, 0, 255);
  data.leftSwitchPosition1 = digitalRead(LEFT_SWITCH_1);
  data.leftSwitchPosition2 = digitalRead(LEFT_SWITCH_2);
  data.rightSwitchPosition1 = digitalRead(RIGHT_SWITCH_1);
  data.rightSwitchPosition2 = digitalRead(RIGHT_SWITCH_2);

  printData(data);

  // todo If the right switch changes, set the radio address re-initialize it.

  radio.write(&data, sizeof(ControlData));
  delay(250);
}

/**
 * Initialize the radio communications.
 */
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
    Serial.print(" Dial: ");
    Serial.print(toPrint.dial);
    Serial.print(" Left Switch 1: ");
    Serial.print(toPrint.leftSwitchPosition1);
    Serial.print(" Left Switch 2: ");
    Serial.print(toPrint.leftSwitchPosition2);
    Serial.print(" Right Switch 1: ");
    Serial.print(toPrint.rightSwitchPosition1);
    Serial.print(" Right Switch 2: ");
    Serial.print(toPrint.rightSwitchPosition2);
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
