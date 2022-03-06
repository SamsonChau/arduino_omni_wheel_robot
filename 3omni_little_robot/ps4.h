#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb);
//PS4BT PS4(&Btd, PAIR);    //for pairing
PS4BT PS4(&Btd);

bool button[16];
int8_t axis[6];
int8_t lp_filter = 80;
float softening = 0.2;

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
