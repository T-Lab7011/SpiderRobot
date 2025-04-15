#include <SPI.h>  // ライブラリのインクルード
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

RF24 radio(8, 9);                 // CE,CSNピンの指定
const byte address[6] = "00001";  // データを送////信するアドレス

//========================================Variables to reading gyro values=============================================
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
//=====================================================================================================================

#define INTERRUPT_PIN 2

MPU6050 mpu;

#define stick_Xpin A7
#define stick_Ypin A1
#define stick_SWpin A2
#define SW1pin 6
#define SW2pin 5
#define SW3pin 7
#define SW4pin 4
#define posSW_pin1 3
#define posSW_pin2 A0
#define Potentiometer1_pin A3

struct PacketData {
  int Joy_X;
  int Joy_Y;
  int Joy_SW;
  int SW_A;
  int SW_B;
  int SW_C;
  int SW_D;
  int Toggle1_SW;
  int Toggle2_SW;
  int Pot1;
  int roll;
  int pitch;
};

PacketData data;

void setup() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(38400);

  while (!Serial)
    ;

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  //You need to set your own offset values to get stable output. You should find this value by try and fail.
  mpu.setXGyroOffset(128);
  mpu.setYGyroOffset(5);
  mpu.setZGyroOffset(-2);
  mpu.setXAccelOffset(-3264);
  mpu.setYAccelOffset(-2064);
  mpu.setZAccelOffset(332);
  //[-3265,-3264] --> [-11,6]	[-2065,-2064] --> [-5,12]	[331,332] --> [16368,16389]	[127,128] --> [-2,2]	[4,5] --> [-1,3]	[-3,-2] --> [0,3]

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  //radio.stopListening();

  pinMode(SW1pin, INPUT_PULLUP);
  pinMode(SW2pin, INPUT_PULLUP);
  pinMode(SW3pin, INPUT_PULLUP);
  pinMode(SW4pin, INPUT_PULLUP);
  pinMode(stick_SWpin, INPUT_PULLUP);
  pinMode(posSW_pin1, INPUT_PULLUP);
  pinMode(posSW_pin2, INPUT_PULLUP);
}

void loop() {
  if (!dmpReady) return;

  readMPU6050();
  // データの読み取り

  data.Joy_X = analogRead(stick_Xpin);         // X軸の値
  data.Joy_Y = analogRead(stick_Ypin);         // Y軸の値
  data.Joy_SW = digitalRead(stick_SWpin);      // スティックのスイッチ
  data.SW_A = digitalRead(SW1pin);             // SW1
  data.SW_B = digitalRead(SW2pin);             // SW2
  data.SW_C = digitalRead(SW3pin);             // SW3
  data.SW_D = digitalRead(SW4pin);             // SW4
  data.Toggle1_SW = digitalRead(posSW_pin1);   // 3ポジションスイッチのピン1
  data.Toggle2_SW = digitalRead(posSW_pin2);   // 3ポジションスイッチのピン2
  data.Pot1 = analogRead(Potentiometer1_pin);  // ポテンショメータ
  data.roll = ypr[1];
  data.pitch = ypr[2];
  radio.write(&data, sizeof(PacketData));
  delay(1000);

  /*
  // データの表示
  Serial.print("X : ");
  Serial.print(data.Joy_X);
  Serial.print("\tY : ");
  Serial.print(data.Joy_Y);
  Serial.print("\tStick Switch : ");
  Serial.print(data.Joy_SW);
  Serial.print("\tSW1 : ");
  Serial.print(data.SW_A);
  Serial.print("\tSW2 : ");
  Serial.print(data.SW_B);
  Serial.print("\tSW3 : ");
  Serial.print(data.SW_C);
  Serial.print("\tSW4 : ");
  Serial.print(data.SW_D);
  Serial.print("\t3posSW Pin1 : ");
  Serial.print(data.Toggle1_SW);
  Serial.print("\t3posSW Pin2 : ");
  Serial.print(data.Toggle2_SW);
  Serial.print("\tPotentiometer : ");
  Serial.print(data.Pot1);
  Serial.print("\troll : ");
  Serial.print(data.roll);
  Serial.print("\tpitch : ");
  Serial.println(data.pitch);
  */
}


void readMPU6050() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = 0;  //ypr[0] = ypr[0] * 180 / M_PI
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
  }
}
