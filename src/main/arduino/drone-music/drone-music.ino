
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "pitches.h"

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

const byte address[6] = "200000"; // Has to match the address of the transmitter.


// The song notes are in here.
int melody[] = {
  NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
  NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
  NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4,
  NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_E4, NOTE_F4,
  NOTE_G4, NOTE_C4,
  
  NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
  NOTE_D4,
  NOTE_F4, NOTE_AS3,
  NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3,
  NOTE_DS4, NOTE_D4, NOTE_C4,
  
  NOTE_G4, NOTE_C4,
  
  NOTE_DS4, NOTE_F4, NOTE_G4, NOTE_C4, NOTE_DS4, NOTE_F4,
  NOTE_D4,
  NOTE_F4, NOTE_AS3,
  NOTE_DS4, NOTE_D4, NOTE_F4, NOTE_AS3,
  NOTE_DS4, NOTE_D4, NOTE_C4,
  NOTE_G4, NOTE_C4,
  NOTE_DS4, NOTE_F4, NOTE_G4,  NOTE_C4, NOTE_DS4, NOTE_F4,
  
  NOTE_D4,
  NOTE_F4, NOTE_AS3,
  NOTE_D4, NOTE_DS4, NOTE_D4, NOTE_AS3,
  NOTE_C4,
  NOTE_C5,
  NOTE_AS4,
  NOTE_C4,
  NOTE_G4,
  NOTE_DS4,
  NOTE_DS4, NOTE_F4,
  NOTE_G4,
  
  NOTE_C5,
  NOTE_AS4,
  NOTE_C4,
  NOTE_G4,
  NOTE_DS4,
  NOTE_DS4, NOTE_D4,
  NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4, NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4,
  NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4, NOTE_C5, NOTE_G4, NOTE_GS4, NOTE_AS4,
  
  REST, NOTE_GS5, NOTE_AS5, NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5,
  NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5, NOTE_C6, NOTE_G5, NOTE_GS5, NOTE_AS5
};

// The note lengths are in here.
int durations[] = {
  8, 8, 16, 16, 8, 8, 16, 16,
  8, 8, 16, 16, 8, 8, 16, 16,
  8, 8, 16, 16, 8, 8, 16, 16,
  8, 8, 16, 16, 8, 8, 16, 16,
  4, 4,
  
  16, 16, 4, 4, 16, 16,
  1,
  4, 4,
  16, 16, 4, 4,
  16, 16, 1,
  
  4, 4,
  
  16, 16, 4, 4, 16, 16,
  1,
  4, 4,
  16, 16, 4, 4,
  16, 16, 1,
  4, 4,
  16, 16, 4, 4, 16, 16,
  
  2,
  4, 4,
  8, 8, 8, 8,
  1,
  2,
  2,
  2,
  2,
  2,
  4, 4,
  1,
  
  2,
  2,
  2,
  2,
  2,
  4, 4,
  8, 8, 16, 16, 8, 8, 16, 16,
  8, 8, 16, 16, 8, 8, 16, 16,
  
  4, 16, 16, 8, 8, 16, 16,
  8, 16, 16, 16, 8, 8, 16, 16
};



/**
 * All setup routines go in here, this is called once on initialzation.
 */
void setup() {

  // Setup the serial communication.
  Serial.begin(9600);
  Serial.print("Initializing...");

  // Output the buzzer.
  pinMode(BUZZER, OUTPUT);

  initRadioComms();

  Serial.print("Done...");
}


/**
 * This method is called continually while the drone runs. It runs as fast as it can.
 */
void loop() {
  
  Serial.println("Waiting for data...");

  if (radio.available()) {
    radio.read(&data, sizeof(data));
    printData(data);

    
    // Turn the LED on if AUX1 is set to 1, turn it off otherwise.
    if(data.AUX1 == 1) {
      Serial.println("Playing a song.");
      playSong();
    } 
  }

  // Wait 250ms so we can see what is going on.
  delay(250);
}


void playSong() {

  int size = sizeof(durations) / sizeof(int);

  for (int note = 0; note < size; note++) {
    //to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int duration = 1000 / durations[note];
    tone(BUZZER, melody[note], duration);

    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);

    //stop the tone playing:
    noTone(BUZZER);
  }
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

/**
 * Print the data raed from the controller.
 */
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