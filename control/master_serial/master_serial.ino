#include <SoftwareSerial.h>

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

SoftwareSerial FR(6, 7); // RX, TX
SoftwareSerial FL(9, 8); // RX, TX
SoftwareSerial RR(10,11); // RX, TX
SoftwareSerial RL(12, 13); // RX, TX

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  // set the data rate for the SoftwareSerial port
  FR.begin(9600);
  FL.begin(9600);
  RR.begin(9600);
  RL.begin(9600);
}

void loop() { // run over and over
// print the string when a newline arrives:
  if (stringComplete) {
    if (inputString.substring(0, 4) == "HOME") {
      FR.println("T002002022\n");
      delay(10);
      FL.println("T002002002\n");
      delay(10);
      RR.println("T002002002\n");
      delay(10);
      RL.println("T002002002\n");

      delay(2000);
      FR.println("T000000000\n");
      delay(10);
      FL.println("T000000000\n");
      delay(10);
      RR.println("T000000000\n");
      delay(10);
      RL.println("T000000000\n");
      delay(2000);
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}