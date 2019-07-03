#include "robot.h"

// Dynamixel Setup //
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1) <-OpenCM9.04
Dynamixel dxl(DXL_BUS_SERIAL1);

Robot minirhex(&dxl);

const int NUM_GAITS = 11;
int gaits[NUM_GAITS] = {STAND, WALK, STAND, LEFT, STAND, WALK, STAND, LEFT, STAND, WALK, STAND};
int durations[NUM_GAITS] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
int time_idx = 0;

// Button Setup //
int button_state;
int last_button_state = 0;

void handle_button_press() {
    button_state = digitalRead(BOARD_BUTTON_PIN);
    if (button_state > last_button_state) {
        digitalWrite(BOARD_LED_PIN, LOW); // turn led on
        // int new_gait_idx = minirhex.incrementGait(); // change to next gait
        // SerialUSB.println(new_gait_idx);
    }
    else if (button_state < last_button_state) {
        digitalWrite(BOARD_LED_PIN, HIGH); // turn led off
    }
    last_button_state = button_state;
}

bool new_gait_started = false;
int t_start = 0;

void setup() {
    minirhex.setup();
    Serial2.begin(57600); // set up serial usb input
    pinMode(BOARD_BUTTON_PIN, INPUT_PULLDOWN); // setup user button
    pinMode(BOARD_LED_PIN, OUTPUT); // setup LED
}

int count = 0;
void loop() {
    SerialUSB.println(millis() - t_start);
    SerialUSB.println(gaits[time_idx]);

    if (!new_gait_started) {
        t_start = millis();
        new_gait_started = true;
    }

    if (millis() - t_start >= durations[time_idx]) {
        time_idx++;
        if (time_idx >= NUM_GAITS) {
            time_idx = NUM_GAITS - 1;
        }
        new_gait_started = false;
        minirhex.updateGait(all_gaits[gaits[time_idx]]);
    }

    //time count
    count++;

    //Every 100 loop iterations, find max voltage supplied to each leg and compare with nominal
    // if (count % 10 == 0) {
    //   SerialUSB.println(minirhex.checkBattery());
    // }

    //button control
    handle_button_press();

    minirhex.move();
    minirhex.checkForBT();
}
