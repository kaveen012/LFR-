#include <QTRSensors.h>
#include <BluetoothSerial.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

int ENA = 23;  //Assign the motor pins
int IN1 = 22;
int IN2 = 21;
int IN3 = 19;
int IN4 = 18;
int ENB = 5;

int P;
int I;
int D;

// PID contorller define
float Kp = 0;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kifinal;
uint8_t Kpfinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;
const uint16_t threshold = 500;

//motor speeds
const int maxspeeda = 150;
const int maxspeedb = 150;
const int basespeeda = 100;
const int basespeedb = 100;

const int minspeeda = -100;
const int minspeedb = -100;

#define LED_BUILTIN 2

QTRSensors qtr;
int lastError = 0;
boolean onoff = 0;
int val, cnt = 0, v[3];

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


void setup() {
  Serial.begin(9600);
  SerialBT.begin("LFR");
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ 35, 32, 33, 25, 26, 27, 14, 12 }, SensorCount);
  qtr.setEmitterPin(3);


  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Wait for calibration to settle
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.print("starting calibrating");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("done calibation");
  delay(500);
  forward_brake(0, 0);
}


void loop() {
  if (Serial.available()) {
    while (Serial.available() == 0)
      ;
    valuesRead();
    processing();
  }
  if (onoff == 0) {
    forward_brake(0, 0);
  }
}

void valuesRead() {
  val = Serial.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

void processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
  if (a == 7) {
    onoff = v[2];
  }
}

void forward_brake(int posa, int posb) {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, HIGH);
  analogWrite(IN1, posa);
  analogWrite(IN3, posb);
}

void left_brake(int posa, int posb) {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB,LOW);
  analogWrite(IN1, posa);
  analogWrite(IN3,posb);
}

void right_brake(int posa, int posb) {
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  analogWrite(IN1, posa);
  analogWrite(IN3, posb);
}

void robot_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  Serial.println(position);
  int error = 3500 - position;
  Serial.println(error);
  int cnt = 0;
  float sum = 0;
  for (int i = 0; i < 8; i++) {
    if(sensorValues[i] >= threshold) {
      cnt++;
      sum = sum + i;
    }
  }

  if (cnt >= 3) {
    int motorspeeda = 0;
    int motorspeedb = 0;
    int val = sum / cnt;
    if (val < 3.5) {
      //turn right
      right_brake(100, 100);
    }
    if (val > 3.5) {
      //turn left
      left_brake(100, 100);
    }
    if (val == 3.5) {
      cnt = cnt / 2;
      uint16_t mini = 1000;
      uint8_t minpos = 0;
      for (int i = 4 - cnt; i <= 3 + cnt; i++) {
        if (mini > sensorValues[i]) {
          mini = sensorValues[i];
          minpos = i;
        }
      }
      if(minpos<3.5){
        right_brake(100, 100);
      }
      if (minpos>3.5){
        left_brake(100, 100);
      }
    }
  }
  else{
    PID(error);
  }
}

void PID(int error) {
int  P = error;
int  I = error + I;
int  D = error - lastError;
  lastError = error;
  Pvalue = (Kp/pow(10,multiP))*P;
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D;
  Serial.println(error);

  float motorspeed = Pvalue + Ivalue + Dvalue;

  int motorspeeda =basespeeda +motorspeed;
  int motorspeedb= basespeedb -motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < minspeeda) {
    motorspeeda = minspeeda;
  }
  if (motorspeedb < minspeedb) {
    motorspeedb = minspeedb;
  }
  //Serial.print(motorspeeda); Serial.print(" "); Serial.println(motorspeedb);
  speed_control(motorspeeda, motorspeedb);
}

void speed_control(int mota, int motb){
  if(mota>=0 && motb >=0)
  {
    forward_brake(mota,motb);
  }
  if(mota <0 && motb >= 0)
  {
    mota =0-mota;
    right_brake(mota,motb);

    
  }
  if(mota>=0 && motb<0)
  {
    motb = 0- motb;
    left_brake(mota,motb);
  }
}

