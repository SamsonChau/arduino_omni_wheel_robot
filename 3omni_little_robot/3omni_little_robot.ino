#include <PS4BT.h>
#include <usbhub.h>
#include "operator.h"
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
//PS4BT PS4(&Btd, PAIR);    //for pairing
PS4BT PS4(&Btd);

#include <MotorDriver.h>
uint8_t SerialDebug = 0;

bool button[16];
int8_t axis[6];
int8_t lp_filter = 80;
float softening = 0.2;

MotorDriver motorDriver = MotorDriver(YF_IIC_TB);
int motor1_speed = 0;
int motor2_speed = 0;
int motor3_speed = 0;

int max_speed = 4096;

float x_axis = 0;
float y_axis = 0;
float z_axis = 0;

float centre_radius = 105;

void ps4_update() {
  //analog button
  axis[0] = PS4.getAnalogButton(L2);
  axis[1] = PS4.getAnalogButton(R2);

  if (PS4.getAnalogHat(LeftHatX) < 0) {
    axis[2] = -(PS4.getAnalogHat(LeftHatX)) - 126;
  }
  else {
    axis[2] = 127 - (PS4.getAnalogHat(LeftHatX));
  }
  if (PS4.getAnalogHat(LeftHatY) < 0) {
    axis[3] = (-(PS4.getAnalogHat(LeftHatY)) - 126);
  }
  else {
    axis[3] = 127 - (PS4.getAnalogHat(LeftHatY));
  }
  if (PS4.getAnalogHat(RightHatX) < 0) {
    axis[4] = (-(PS4.getAnalogHat(RightHatX)) - 126);
  }
  else {
    axis[4] = 127 - (PS4.getAnalogHat(RightHatX));
  }

  if (PS4.getAnalogHat(RightHatY) < 0) {
    axis[5] = (-(PS4.getAnalogHat(RightHatY)) - 126);
  }
  else {
    axis[5] = 127 - (PS4.getAnalogHat(RightHatY));
  }
  //buttonstate
  if (PS4.getButtonClick(UP)) {
    button[0] = !button[0];
  }
  if (PS4.getButtonClick(DOWN)) {
    button[1] = !button[1];
  }
  if (PS4.getButtonClick(LEFT)) {
    button[2] = !button[2];
  }
  if (PS4.getButtonClick(RIGHT)) {
    button[3] = !button[3];
  }
  if (PS4.getButtonClick(SQUARE)) {
    button[4] = !button[4];
  }
  if (PS4.getButtonClick(CROSS)) {
    button[5] = !button[5];
  }
  if (PS4.getButtonClick(CIRCLE)) {
    button[6] = !button[6];
  }
  if (PS4.getButtonClick(TRIANGLE)) {
    button[7] = !button[7];
  }
  if (PS4.getButtonClick(SHARE)) {
    button[8] = !button[8];
  }
  if (PS4.getButtonClick(OPTIONS)) {
    button[9] = !button[9];
  }
  if (PS4.getButtonClick(PS)) {
    button[10] = !button[10];
  }
  if (PS4.getButtonClick(TOUCHPAD)) {
    button[11] = !button[11];
  }
  //buttonstate
  if (PS4.getButtonClick(L1)) {
    button[12] = !button[12];
    if (softening <= 0.8)
    {
      softening += 0.2;
      Serial.println("Softenvalue:");
      Serial.print(softening);
    }
  }
  if (PS4.getButtonClick(R1)) {
    button[13] = !button[13];
    if (softening >= 0.05)
    {
      softening -= 0.2;
      Serial.println("Softenvalue:");
      Serial.print(softening);
    }
  }
  if (PS4.getButtonClick(L3)) {
    button[14] = !button[14];
  }
  if (PS4.getButtonClick(R3)) {
    button[15] = !button[15];
  }
  //vabration
  //PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
  //PS4.setRumbleOn(RumbleLow);
  //PS4.setRumbleOn(RumbleHigh);

  //blink
  //PS4.setLedFlash(10, 10);
  //PS4.setLedFlash(0, 0);
  //PS4.setLed(Red);
  //PS4.setLed(Yellow);
  //PS4.setLed(Blue);
  //PS4.setLed(Green);

  //PS4.disconnect();
}

void setup() {
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
    x_axis =  filter_low(soften_value(map(axis[4], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);
    y_axis =  filter_low(soften_value(map(axis[5], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);
    z_axis =  filter_low(soften_value(map(axis[2], -128, 127, -max_speed, max_speed), softening, max_speed), lp_filter);

    motor1_speed = (-x_axis * 0.33 + y_axis * 0.58 + z_axis * 0.33);
    motor2_speed = (-x_axis * 0.33 - y_axis * 0.58 + z_axis * 0.33);
    motor3_speed = (x_axis * 0.67  + y_axis * 0.0  + z_axis * 0.33);

    Serial.print("\r\nmotor1: ");
    Serial.print(motor1_speed);
    Serial.print(" motor2: ");
    Serial.print(motor2_speed);
    Serial.print(" motor3: ");
    Serial.print(motor3_speed);

//    Serial.print("\r\nx: ");
//    Serial.print(x_axis);
//    Serial.print(axis[4]);
//    Serial.print(" y: ");
//    Serial.print(y_axis);
//    Serial.print(axis[5]);
//    Serial.print(" z: ");
//    Serial.print(z_axis);
//    Serial.print(axis[2]);

    if (button[11]) {
      motorDriver.setMotor(motor1_speed, motor2_speed, motor3_speed, 0);
    }
    else {
      motorDriver.setAllMotor(0);   // all motor stop
      motorDriver.stopMotor(MAll);  // all motors brake
    }
  }
}
