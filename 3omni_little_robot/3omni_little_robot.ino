//#include <util/atomic.h>
#include "ps4.h"
#include "operator.h"
//#include "pid.h"

#include <MotorDriver.h>
uint8_t SerialDebug = 0;

MotorDriver motorDriver = MotorDriver(YF_IIC_TB);
int motor1_speed = 0;
int motor2_speed = 0;
int motor3_speed = 0;

int max_speed = 4096;

float x_axis = 0;
float y_axis = 0;
float z_axis = 0;



void setup() {
  //  pinMode(ENC1A, INPUT);
  //  pinMode(ENC1B, INPUT);
  //  pinMode(ENC2A, INPUT);
  //  pinMode(ENC2B, INPUT);
  //  pinMode(ENC3A, INPUT);
  //  pinMode(ENC3B, INPUT);
  //
  //  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  //  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
  //  attachInterrupt(digitalPinToInterrupt(ENC3A), readEncoder3, RISING);

  Serial.begin(115200);
  Serial.println("\r\n3 omni wheel robot start");
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start, debug the hardware connection."));
    while (1);
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started, press the logo for connection or press teh logo with share to pair the controller"));
  motorDriver.begin();
  motorDriver.motorConfig(1, 1, 1, 1);
  Serial.println("\r\nStart...");
}

void loop() {
  Usb.Task();
  if (PS4.connected()) {
    ps4_update();
    x_axis =  map(axis[4], -128, 127, -max_speed, max_speed)*2;
    y_axis =  map(axis[5], -128, 127, -max_speed, max_speed)*2;
    z_axis =  map(axis[2], -128, 127, -max_speed, max_speed)*2;

    //    x_axis =  filter_low(soften_value(map(axis[4], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);
    //    y_axis =  filter_low(soften_value(map(axis[5], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);
    //    z_axis =  filter_low(soften_value(map(axis[2], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);
//
//    motor1_speed = (int)(-x_axis / 3 + y_axis * 0.58 + z_axis / 3);
//    motor2_speed = (int)(-x_axis / 3 - y_axis * 0.58 + z_axis / 3);
//    motor3_speed = (int)(x_axis * 0.67 + z_axis / 3);
    
    motorDriver.setMotor(x_axis,y_axis,z_axis,0);
    //motorDriver.setMotor(setrpm(motor1_speed, 1), setrpm(motor2_speed, 2), setrpm(motor3_speed, 3), 0);
    //    Serial.print("\r\nmotor1: ");
    //    Serial.print(motor1_speed);
    //    Serial.print(" motor2: ");
    //    Serial.print(motor2_speed);
    //    Serial.print(" motor3: ");
    //    Serial.print(motor3_speed);

    //    Serial.print("\r\nx: ");
    //    Serial.print(x_axis);
    //    Serial.print(axis[4]);
    //    Serial.print(" y: ");
    //    Serial.print(y_axis);
    //    Serial.print(axis[5]);
    //    Serial.print(" z: ");
    //    Serial.print(z_axis);
    //    Serial.print(axis[2]);
  }
//  Serial.print("\r\nenc1: ");
//  Serial.print(ENC1_pos);
//  Serial.print(" enc2: ");
//  Serial.print(ENC2_pos);
//  Serial.print(" enc3: ");
//  Serial.print(ENC3_pos);
//
//  Serial.print("motor1: ");
//  Serial.print(setrpm(motor1_speed, 1));
//  Serial.print(" motor2: ");
//  Serial.print(setrpm(motor2_speed, 2));
//  Serial.print(" motor3: ");
//  Serial.print(setrpm(motor3_speed, 3));
}
