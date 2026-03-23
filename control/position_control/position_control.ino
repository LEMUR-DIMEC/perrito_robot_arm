#include "spi_msg.h"
#include "drv.h"


String inputString = "";
bool stringComplete = false;

uint8_t hombro_target = 0;
uint8_t hombro_pos = 0;
uint8_t sup_target = 0;
uint8_t sup_pos = 0;
uint8_t inf_target = 0;
uint8_t inf_pos = 0;

void serialEvent() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
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
  // put your setup code here, to run once:
  Serial1.begin(9600);
  delay(2000);
  drv_setup();
  spi_master_init();  // setup once
  delay(2000);
}

void loop() {
  // main FOC algorithm function
  motor_loop();
  motor_move();
  serialEvent();
  // put your main code here, to run repeatedly:

  if (stringComplete) {
    if (inputString.substring(0, 1) == "T") {
      update_target(hombro_target);
      hombro_target = inputString.substring(1, 4).toInt();
      sup_target = inputString.substring(4, 7).toInt();
      inf_target = inputString.substring(7, 11).toInt();
      if (spi_is_writable(spi_default)){
      sup_pos=spi_exchange_angle(CS_SLAVE1, sup_target);
      inf_pos=spi_exchange_angle(CS_SLAVE2, inf_target);
      }
      else{
        digitalWrite(TEMP_LED, HIGH);
      }
      inputString = "";
      stringComplete = false;
    }
    inputString = "";
    stringComplete = false;
    
  }
}
