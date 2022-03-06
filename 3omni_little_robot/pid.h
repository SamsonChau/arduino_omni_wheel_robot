// Pins
#define ENC1A 2
#define ENC1B 4
#define ENC2A 3
#define ENC2B 5
#define ENC3A 6
#define ENC3B 7

// PID constants
float kp = 1;
float kd = 0.025;
float ki = 0.0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

long ENC1_pos = 0;
long ENC2_pos = 0;
long ENC3_pos = 0;

long ENC1_prepos = 0;
long ENC2_prepos = 0;
long ENC3_prepos = 0;

int setrpm(float set_point, int mode) {
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;
  float enc = 0;
  if (mode == 1) {
    enc = (ENC1_pos - ENC1_prepos) /448 /18.8 * 60 / deltaT;
  }
  else if (mode == 2) {
    enc = (ENC2_pos - ENC2_prepos) /448 /18.8 * 60 / deltaT;
  }
  else if (mode == 3 ) {
    enc = (ENC3_pos - ENC3_prepos) /448 /18.8 * 60 / deltaT;
  }
  else {
    enc = 0;
  }
  int e = enc  - set_point;
  float dedt = (e - eprev) / (deltaT);
  eintegral = eintegral + e * deltaT;
  float u = kp * e + kd * dedt + ki * eintegral;
  return (int)u;                                                 //have function return the PID output
}

void readEncoder1() {
  if (digitalRead(ENC1B)) {
    ENC1_pos++;
  }
  else {
    ENC1_pos--;
  }
}

void readEncoder2() {
  if (digitalRead(ENC2B)) {
    ENC2_pos++;
  }
  else {
    ENC2_pos--;
  }
}

void readEncoder3() {
  if (digitalRead(ENC3B)) {
    ENC3_pos++;
  }
  else {
    ENC3_pos--;
  }
}
