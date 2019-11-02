#include "robot.h"
#include "conversions.h"

Robot::Robot(Dynamixel * dxl) : Dxl(dxl) {
    legs_active = 6;
    packet_length = 2*legs_active;

    legs[0] = Leg(1, 0, stand_gait, 0, false, false, false);
    legs[1] = Leg(2, 0, stand_gait, 0, false, false, false);
    legs[2] = Leg(3, 0, stand_gait, 0, false, false, false);
    legs[3] = Leg(4, 0, stand_gait, 0,  true, false, false);
    legs[4] = Leg(5, 0, stand_gait, 0,  true, false, false);
    legs[5] = Leg(6, 0, stand_gait, 0,  true, false, false);
}

void Robot::setup() {
    Dxl->begin(3); // baudrate set to 1 Mbps (max)
    uint64_t t_start = millis();
    for (int i = 0; i < legs_active; i++) { // legs stored at their index
        legs[i].updateGait(stand_gait, t_start); // set initial parameters, initial_gait in gait_parameters
    }
    Dxl->writeWord( BROADCAST_ID, 30, 0 );
    Dxl->writeWord( BROADCAST_ID, 32, 0 );
}

int Robot::incrementGait() {
    gait_idx = (gait_idx + 1) % TOTAL_GAITS;
    return updateGait(gait_order[gait_idx]);
}

int Robot::updateGait(Gait gait) {
    int t_start = millis();
    for(int i = 0; i < legs_active; i++) {
      legs[i].updateGait(gait, t_start);
    }
    return gait_idx;
}

unsigned short Robot::checkBattery() {
  for (int i = 0; i < legs_active; i++) {
    uint8_t voltage_check = Dxl->getVolt(legs[i].id);
    if (voltage_check > voltage) {
      voltage = voltage_check;
    }
  }
  
  Serial.print("Voltage: ");
  Serial.println(voltage);

  if (voltage > 73) { // green
    low_battery = 2;
  }
  else if (voltage < 71) { // red
    low_battery = 1;
  }
  else {
    low_battery = 3; // yellow
  }

  if (prev_low_battery != low_battery) {
    Serial.println("Voltage Low!");
    for (int i = 0; i < legs_active; i++) {
      Dxl->writeByte(legs[i].id, LED, low_battery);
    }
  }
  return voltage;
}

float Robot::pd_calc(float theta_act, float theta_des,
                        float v_act, float v_des,
                        float kp, float kd)
{
  float diff = fmod(theta_des - theta_act, theta_circle);
  float shortest_distance = theta_circle/2 - fabs(fabs(diff) - theta_circle/2);
  float dtheta = fmodf(diff + theta_circle, theta_circle) < theta_circle/2 ? shortest_distance : -shortest_distance;
  float dv = (v_des - v_act);
  return kp * dtheta + kd * dv;
}

void Robot::update() {    
    word packet[packet_length];

    // primary for-loop
    for(int i = 0; i < legs_active; i++) {
      packet[2*(i-1)] = legs[i].id;
      actual_p = Dxl->readWord(legs[i].id, PRESENT_POS);
      actual_theta = P_to_Theta(actual_p); // converted to degrees, relative to leg
      actual_vel = dynV_to_V(Dxl->readWord(legs[i].id, PRESENT_SPEED)); // converted to degrees/ms, relative to leg
  
      if (!legs[i].deadzone) {
        if (actual_p == 0 || actual_p == 1023) { //entering deadzone
          legs[i].deadzone = true;
          if (actual_p == 0) legs[i].dead_from_neg = true;
          else legs[i].dead_from_neg = false;
          continue;
        }

        if (legs[i].gait.id == 0) { // standing or sitting
          if (legs[i].right_side) {
              desired_theta = Theta_to_ThetaR(legs[i].desired_theta);
          }
          else {
              desired_theta = legs[i].desired_theta;
          }
          actual_theta = actual_theta - legs[i].zero; //zero out leg thetas, accounts for small servo irregularities
          control_signal = pd_calc(actual_theta, desired_theta, actual_vel, 0, kp_hold, kd_hold);
        }
        else { // walking, turning
          // compute absolute desired values (theta and velocity) from clock time
          legs[i].getDesiredVals(millis());
          // translate theta and v to relative (left and right)
          if (legs[i].right_side) {
            desired_vel = -legs[i].global_velocity; //relative
            desired_theta = Theta_to_ThetaR(legs[i].global_theta); // relative
          }
          else { // left side, relative is same as global
            desired_vel = legs[i].global_velocity;
            desired_theta = legs[i].global_theta;
          }
          actual_theta = actual_theta - legs[i].zero;
          control_signal = pd_calc(actual_theta, desired_theta, actual_vel, desired_vel, legs[i].kp, legs[i].kd);
      }
  
      int new_vel = V_to_dynV(actual_vel + control_signal);
        packet[2*(i-1) + 1] = new_vel;
      }
  
      else { //deadzone
        if ((actual_p > 0) & (actual_p < dead_buffer) || (actual_p < 1023) & (actual_p > 1023 -dead_buffer)) { //exiting deadzone
          legs[i].deadzone = false;
        }
        float signed_recovery_speed = legs[i].dead_from_neg ? -legs[i].recovery_speed : legs[i].recovery_speed;
        packet[2*(i-1) + 1] = V_to_dynV(signed_recovery_speed);
      }
    }
    
//    Serial.print("[ ");
//    for (int i = 0; i < packet_length; i++){
//      Serial.print(packet[i]);
//      Serial.print(" ");
//    }
//    Serial.print("]");
//    Serial.println();
    
    Dxl->syncWrite(MOVING_SPEED, 1, packet, packet_length); //simultaneously write to each of 6 servoes with updated commands
}

void Robot::checkForBT() {
  // bluetooth control
  if (Serial2.available()) {
    char a = (char)(Serial2.read());
    Serial.println(a);
    int bt_gait_idx = -1;
    switch (a) {
    case 'q':
      bt_gait_idx = 0;
      break; //stand
    case 'w':
      bt_gait_idx = 1;
      break; //forwards
    case 's':
      bt_gait_idx = 2;
      break; //reverse
    case 'a':
      bt_gait_idx = 3;
      break; //left
    case 'd':
      bt_gait_idx = 4;
      break; //right
    case 'e':
      bt_gait_idx = 5;
      break;
    case 'x':
      jumpReady();
      break;
    case 'j':
      jump();
      break;
    }

    if (bt_gait_idx != -1) {
      int t_start = millis();
      gait_idx = bt_gait_idx;
      updateGait(gait_order[gait_idx]);
    }
  }
}

void Robot::printServoPositions() {
  for (int i = 0; i < legs_active; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(P_to_Theta(Dxl->readWord(i, PRESENT_POS)));
  }
}
