#include "robot.h"
#include <Dynamixel.h>

// Dynamixel Setup //
#define DXL_BUS_SERIAL1 1  // Dynamixel on Serial1(USART1) <-OpenCM9.04
Dynamixel Dxl(DXL_BUS_SERIAL1);

Robot MiniRHex(&Dxl);

// Button Setup //
unsigned char button_state;
unsigned char last_button_state = 0;
unsigned long button_t = millis();

void handle_button_press() {
  button_state = digitalRead(BOARD_BUTTON_PIN);
  if (button_state > last_button_state) {
    if (millis() - button_t > 1000) {
      button_t = millis();
      digitalWrite(BOARD_LED_PIN, LOW); // turn led on
      int new_gait_idx = MiniRHex.incrementGait(); // change to next gait
    }
//    Serial.println(new_gait_idx);
  }
  else if (button_state < last_button_state) {
    digitalWrite(BOARD_LED_PIN, HIGH); // turn led off
  }
  last_button_state = button_state;
}

void setup() {
  MiniRHex.startup();
  Serial2.begin(57600);
  pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN); // setup user button
  pinMode(BOARD_LED_PIN, OUTPUT); // setup LED
}

unsigned long t = millis();
unsigned long u_t = millis();
void loop() {
  // Every second, find max voltage supplied to each leg and compare with nominal // 
  if (millis() - t > 1000) {
    t = millis();
//    MiniRHex.checkBattery();
  }

  // button control //
  handle_button_press();

  if (millis() - u_t > 10) {
    u_t = millis();
    MiniRHex.update();
//    int packet[] =  {1, 1023, 2, 1023, 3, 1023, 4, 1023, 5, 1023, 6, 1023};
//    MiniRHex.Dxl->syncWrite(MOVING_SPEED, 1, packet, 12);
//    MiniRHex.printServoPositions();
  }
//  MiniRHex.checkForBT();
}
