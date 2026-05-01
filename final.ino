#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

uint8_t IDs[]={1, 2, 101, 7, 3, 6}; // Leg ID corresponding to [LF1, LF2, RF1, RF2, LR, RR] legs.
uint8_t Directions[]={1, 0, 0, 1, 0, 1}; // Leg rotating direction corresponding to [LF1, LF2, RF1, RF2, LR, RR]; value=0 then rotate CCW, value=1 then rotate CW

int Leg_zeroing_offset[]={150, 240, 150, 60, 150, 60};
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////

long start;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<4;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  dxl.torqueOff(IDs[4]);
  dxl.setOperatingMode(IDs[4], OP_VELOCITY);
  dxl.torqueOn(IDs[4]);

  dxl.torqueOff(IDs[5]);
  dxl.setOperatingMode(IDs[5], OP_VELOCITY);
  dxl.torqueOn(IDs[5]);

  delay(5000);

  start = millis();
}

long last_time=0;
int time_step=100;
int counter = 0;

void loop() {
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  long elapsed = millis() - start;

  // Have rear "whegs" running at these speed constantly
  dxl.setGoalVelocity(IDs[4], 1024 + 500);
  dxl.setGoalVelocity(IDs[5], 400);
  
  
  // Motor angles derived from priscilla_trajectory.py
  int theta_1[] = {41, 31, 23, 17, 11, 6, 1, -3, -7, -10, -13, -15, -16, -16, -15, -13, -10, -6, -1, 5, 11, 18, 24, 29, 35, 40, 44, 48, 52, 56, 59, 57, 55, 52, 50, 48, 46, 44, 42, 41, 39, 38, 36, 35, 34, 33, 32, 31, 31, 30, 30, 30, 30, 30, 31, 32, 33, 35, 37, 41};
  
  int theta_2[] = {20, 36, 47, 56, 65, 73, 80, 87, 94, 100, 106, 111, 116, 120, 124, 127, 130, 131, 131, 131, 129, 126, 123, 119, 114, 109, 103, 97, 91, 84, 77, 78, 79, 79, 79, 80, 80, 80, 79, 79, 78, 77, 77, 75, 74, 73, 71, 69, 67, 65, 62, 60, 57, 53, 50, 46, 41, 35, 29, 20};
  
  if (elapsed-last_time>time_step){
    counter++;
    DEBUG_SERIAL.println(counter);

    last_time=elapsed;
    for (int i=0; i < 60; i++) {

        // WALK Gait
        int t = 0;
        if (i > 29) t = i - 30;
        else t = i + 30;
        int desired_pos_1 = theta_1[59 - i];
        int desired_pos_2 = theta_2[59 - i];
        int desired_pos_3 = theta_1[59 - t];
        int desired_pos_4 = theta_2[59 - t];

        if (Directions[0]==0) {
          desired_pos_1= 360.0 - desired_pos_1;
        }

        if (Directions[1]==0) {
          desired_pos_2= 360.0 - desired_pos_2;
        }

        if (Directions[2]==0) {
          desired_pos_3= 360.0 - desired_pos_3;
        }

        if (Directions[3]==0) {
          desired_pos_4= 360.0 - desired_pos_4;
        }

        desired_pos_1 = desired_pos_1 + Leg_zeroing_offset[0];
        desired_pos_2 = desired_pos_2 + Leg_zeroing_offset[1];
        desired_pos_3 = desired_pos_3 + Leg_zeroing_offset[2];
        desired_pos_4 = desired_pos_4 + Leg_zeroing_offset[3];

        if (desired_pos_1 >= 360) desired_pos_1 = desired_pos_1 - 360.0;
        if (desired_pos_2 >= 360) desired_pos_2 = desired_pos_2 - 360.0;
        if (desired_pos_3 >= 360) desired_pos_3 = desired_pos_3 - 360.0;
        if (desired_pos_4 >= 360) desired_pos_4 = desired_pos_4 - 360.0;

        dxl.setGoalPosition(IDs[0], desired_pos_1, UNIT_DEGREE);
        dxl.setGoalPosition(IDs[1], desired_pos_2, UNIT_DEGREE);
        dxl.setGoalPosition(IDs[2], desired_pos_3, UNIT_DEGREE);
        dxl.setGoalPosition(IDs[3], desired_pos_4, UNIT_DEGREE);

        delay(15);

    }
  }
  
}
