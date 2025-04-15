#include <SPI.h>  // ライブラリのインクルード
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

RF24 radio(6, 8);                 // CE,CSNピンの指定
const byte address[6] = "00001";  // データを送////信するアドレス
Servo servo[4][3];
const int servo_pin[4][3] = { { A3, A2, A1 }, { A0, 3, 2 }, { 9, 7, 5 }, { 10, A5, A4 } };
int servo_pos_now[4][3];
int stand_pos[4][3] = { { 50, 60, 80 }, { 135, 125, 100 }, { 50, 60, 90 }, { 130, 120, 90 } };
int stand_pos_2[4][3] = { { 50, 60, 90 }, { 135, 125, 90 }, { 50, 60, 80 }, { 130, 120, 100 } };
int sweep_pos[4][3] = { { 90, 100, 90 }, { 105, 95, 90 }, { 60, 70, 80 }, { 105, 95, 100 } };
int move_speed = 5;  //少ない方が早い
int stride_length[9] = { 9, 18, 27, 36, 45, 54, 63, 72, 81 };
int turn_angle[9] = { 9, 18, 27, 36, 45, 54, 63, 72, 81 };
int angle;
unsigned long currentreceptionTime = 0;
unsigned long lastreceivedTime = 0;
int flag = 0;
int n = 1;

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
  Serial.begin(38400);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  dataReset();

  servo_attach();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      servo[i][j].write(stand_pos[i][j]);
      servo_pos_now[i][j] = stand_pos[i][j];
    }
  }
  delay(1000);

}



void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(PacketData));
    lastreceivedTime = millis();
  }
  currentreceptionTime = millis();
  if (currentreceptionTime - lastreceivedTime > 2000) {
    dataReset();
  }
  /*
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
  Serial.println(data.Pot1);
  */
  int X = data.Joy_X;
  int Y = data.Joy_Y;

  move_speed = map(data.Pot1, 1023, 0, 5, 2);
  if (X > 530 && Y < 610 && Y > 450) forward(move_speed, stride_length[4], &n);
  else if (X < 490 && Y < 610 && Y > 450) backward(move_speed, stride_length[4], &n);
  else if (Y > 550 && X > 470 && X < 550) right_turn(move_speed, turn_angle[4], &n);
  else if (Y < 510 && X > 470 && X < 550) left_turn(move_speed, turn_angle[4], &n);
  else if (data.SW_A == 0) up();
  else if (data.SW_B == 0) sweep_R();
  else if (data.SW_C == 0) down();
  else if (data.SW_D == 0) sweep_L();
  else stop();
}

void servo_attach(void) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}

//-----------------------Forward-------------------------------------
void forward(int speed, int stride_length, int *n) {
  int lugnum[4] = { 2, 1, 3, 0 };
  if (flag != 0) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(stand_pos[i][j]);
        servo_pos_now[i][j] = stand_pos[i][j];
      }
    }
    *n = 1;
    flag = 0;
  }
  leg_forward(lugnum[*n - 1], speed, stride_length);
  ++*n;
  if (*n == 5) *n = 1;
}

void leg_forward(int legnum, int speed, int stride_length) {
  int timing = stride_length / 9;
  for (int i = 1; i <= 40; ++i) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) --servo_pos_now[legnum][1];
    else ++servo_pos_now[legnum][1];
    delay(speed);
    if (i % (40 / timing) == 0) body_forward(legnum);
  }

  for (int i = 1; i <= stride_length; ++i) {
    servo[legnum][2].write(servo_pos_now[legnum][2]);
    if (legnum % 3 == 0) ++servo_pos_now[legnum][2];
    else --servo_pos_now[legnum][2];
    delay(speed);
    if (i % 9 == 0) body_forward(legnum);
  }

  for (int i = 1; i <= 40; ++i) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) ++servo_pos_now[legnum][1];
    else --servo_pos_now[legnum][1];
    delay(speed);
    if (i % (40 / timing) == 0) body_forward(legnum);
  }
}

void body_forward(int num) {
  move_ThirdJoint(servo_pos_now[0][2], servo_pos_now[1][2], servo_pos_now[2][2], servo_pos_now[3][2]);
  --servo_pos_now[0][2];
  ++servo_pos_now[1][2];
  ++servo_pos_now[2][2];
  --servo_pos_now[3][2];
  if (num % 3 == 0) ++servo_pos_now[num][2];
  else --servo_pos_now[num][2];
}

//-----------------------Backward-------------------------------------
void backward(int speed, int stride_length, int *n) {
  int lugnum[4] = { 0, 3, 1, 2 };
  if (flag != 1) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(stand_pos_2[i][j]);
        servo_pos_now[i][j] = stand_pos_2[i][j];
      }
    }
    *n = 1;
    flag = 1;
  }
  leg_backward(lugnum[*n - 1], speed, stride_length);
  ++*n;
  if (*n == 5) *n = 1;
}

void leg_backward(int legnum, int speed, int stride_length) {
  int timing = stride_length / 9;
  for (int i = 1; i <= 40; ++i) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) --servo_pos_now[legnum][1];
    else ++servo_pos_now[legnum][1];
    delay(speed);
    if (i % (40 / timing) == 0) body_backward(legnum);
  }

  for (int i = 1; i <= stride_length; ++i) {
    servo[legnum][2].write(servo_pos_now[legnum][2]);
    if (legnum % 3 == 0) --servo_pos_now[legnum][2];
    else ++servo_pos_now[legnum][2];
    delay(speed);
    if (i % 9 == 0) body_backward(legnum);
  }

  for (int i = 1; i <= 40; ++i) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) ++servo_pos_now[legnum][1];
    else --servo_pos_now[legnum][1];
    delay(speed);
    if (i % (40 / timing) == 0) body_backward(legnum);
  }
}

void body_backward(int num) {
  move_ThirdJoint(servo_pos_now[0][2], servo_pos_now[1][2], servo_pos_now[2][2], servo_pos_now[3][2]);
  ++servo_pos_now[0][2];
  --servo_pos_now[1][2];
  --servo_pos_now[2][2];
  ++servo_pos_now[3][2];
  if (num % 3 == 0) --servo_pos_now[num][2];
  else ++servo_pos_now[num][2];
}

//-----------------------RigtTurn-------------------------------------
void right_turn(int speed, int angle, int *n) {
  if (flag != 2) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(stand_pos_2[i][j]);
        servo_pos_now[i][j] = stand_pos_2[i][j];
      }
    }
    *n = 1;
    flag = 2;
  }
  int legnum[4] = { 0, 3, 2, 1 };
  right_move_leg(legnum[*n - 1], speed, angle);
  ++*n;
  if (*n == 5) *n = 1;
}

void right_move_leg(int legnum, int speed, int angle) {
  int timing = angle / 9;
  for (int j = 1; j <= 40; ++j) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) --servo_pos_now[legnum][1];
    else ++servo_pos_now[legnum][1];
    if (j % (40 / timing) == 0) right_turn_body(legnum);
    delay(speed);
  }
  for (int j = 1; j <= angle; ++j) {
    servo[legnum][2].write(servo_pos_now[legnum][2]);
    --servo_pos_now[legnum][2];
    if (j % 9 == 0) right_turn_body(legnum);
    delay(speed);
  }
  for (int j = 1; j <= 40; ++j) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) ++servo_pos_now[legnum][1];
    else --servo_pos_now[legnum][1];
    if (j % (40 / timing) == 0) right_turn_body(legnum);
    delay(speed);
  }
}

void right_turn_body(int num) {
  move_ThirdJoint(servo_pos_now[0][2], servo_pos_now[1][2], servo_pos_now[2][2], servo_pos_now[3][2]);
  ++servo_pos_now[0][2];
  ++servo_pos_now[1][2];
  ++servo_pos_now[2][2];
  ++servo_pos_now[3][2];
  --servo_pos_now[num][2];
}

//-----------------------LeftTurn-------------------------------------
void left_turn(int speed, int angle, int *n) {
  if (flag != 3) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(stand_pos_2[i][j]);
        servo_pos_now[i][j] = stand_pos_2[i][j];
      }
    }
    *n = 1;
    flag = 3;
  }


  int legnum[4] = { 1, 2, 3, 0 };
  left_move_leg(legnum[*n - 1], speed, angle);
  ++*n;
  if (*n == 5) *n = 1;
}

void left_move_leg(int legnum, int speed, int angle) {
  int timing = angle / 9;
  for (int j = 1; j <= 40; ++j) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) --servo_pos_now[legnum][1];
    else ++servo_pos_now[legnum][1];
    if (j % (40 / timing) == 0) left_turn_body(legnum);
    delay(speed);
  }
  for (int j = 1; j <= angle; ++j) {
    servo[legnum][2].write(servo_pos_now[legnum][2]);
    ++servo_pos_now[legnum][2];
    if (j % 9 == 0) left_turn_body(legnum);
    delay(speed);
  }
  for (int j = 1; j <= 40; ++j) {
    servo[legnum][1].write(servo_pos_now[legnum][1]);
    if (legnum % 2 == 0) ++servo_pos_now[legnum][1];
    else --servo_pos_now[legnum][1];
    if (j % (40 / timing) == 0) left_turn_body(legnum);
    delay(speed);
  }
}

void left_turn_body(int num) {
  move_ThirdJoint(servo_pos_now[0][2], servo_pos_now[1][2], servo_pos_now[2][2], servo_pos_now[3][2]);
  --servo_pos_now[0][2];
  --servo_pos_now[1][2];
  --servo_pos_now[2][2];
  --servo_pos_now[3][2];
  ++servo_pos_now[num][2];
}

//-----------------------Stop-------------------------------------
void stop() {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      servo[i][j].write(servo_pos_now[i][j]);
    }
  }
}


void sweep_R() {
  if (flag != 4) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(sweep_pos[i][j]);
        servo_pos_now[i][j] = sweep_pos[i][j];
      }
    }
    flag = 4;
  }

  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    --servo_pos_now[0][0];
    --servo_pos_now[0][1];
    --servo_pos_now[1][0];
    --servo_pos_now[1][1];
    ++servo_pos_now[2][0];
    ++servo_pos_now[2][1];
    ++servo_pos_now[3][0];
    ++servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    --servo_pos_now[0][0];
    --servo_pos_now[0][1];
    ++servo_pos_now[1][0];
    ++servo_pos_now[1][1];
    ++servo_pos_now[2][0];
    ++servo_pos_now[2][1];
    --servo_pos_now[3][0];
    --servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    ++servo_pos_now[0][0];
    ++servo_pos_now[0][1];
    ++servo_pos_now[1][0];
    ++servo_pos_now[1][1];
    --servo_pos_now[2][0];
    --servo_pos_now[2][1];
    --servo_pos_now[3][0];
    --servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    ++servo_pos_now[0][0];
    ++servo_pos_now[0][1];
    --servo_pos_now[1][0];
    --servo_pos_now[1][1];
    --servo_pos_now[2][0];
    --servo_pos_now[2][1];
    ++servo_pos_now[3][0];
    ++servo_pos_now[3][1];
    delay(15);
  }
}

void sweep_L(){
    if (flag != 4) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 3; ++j) {
        servo[i][j].write(sweep_pos[i][j]);
        servo_pos_now[i][j] = sweep_pos[i][j];
      }
    }
    flag = 4;
  }

  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    ++servo_pos_now[0][0];
    ++servo_pos_now[0][1];
    ++servo_pos_now[1][0];
    ++servo_pos_now[1][1];
    --servo_pos_now[2][0];
    --servo_pos_now[2][1];
    --servo_pos_now[3][0];
    --servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    ++servo_pos_now[0][0];
    ++servo_pos_now[0][1];
    --servo_pos_now[1][0];
    --servo_pos_now[1][1];
    --servo_pos_now[2][0];
    --servo_pos_now[2][1];
    ++servo_pos_now[3][0];
    ++servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    --servo_pos_now[0][0];
    --servo_pos_now[0][1];
    --servo_pos_now[1][0];
    --servo_pos_now[1][1];
    ++servo_pos_now[2][0];
    ++servo_pos_now[2][1];
    ++servo_pos_now[3][0];
    ++servo_pos_now[3][1];
    delay(15);
  }
  for (int i = 0; i < 15; ++i) {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    --servo_pos_now[0][0];
    --servo_pos_now[0][1];
    ++servo_pos_now[1][0];
    ++servo_pos_now[1][1];
    ++servo_pos_now[2][0];
    ++servo_pos_now[2][1];
    --servo_pos_now[3][0];
    --servo_pos_now[3][1];
    delay(15);
  }
}

void up() {
  if (servo_pos_now[0][1] > 140) return;
  else {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    ++servo_pos_now[0][0];
    --servo_pos_now[1][0];
    ++servo_pos_now[2][0];
    --servo_pos_now[3][0];
    ++servo_pos_now[0][1];
    --servo_pos_now[1][1];
    ++servo_pos_now[2][1];
    --servo_pos_now[3][1];
    delay(30);
  }
  flag = 5;
}

void down() {
  if (servo_pos_now[1][0] > 150) return;
  else {
    move_FristJoint(servo_pos_now[0][0], servo_pos_now[1][0], servo_pos_now[2][0], servo_pos_now[3][0]);
    move_SecondJoint(servo_pos_now[0][1], servo_pos_now[1][1], servo_pos_now[2][1], servo_pos_now[3][1]);
    --servo_pos_now[0][0];
    ++servo_pos_now[1][0];
    --servo_pos_now[2][0];
    ++servo_pos_now[3][0];
    --servo_pos_now[0][1];
    ++servo_pos_now[1][1];
    --servo_pos_now[2][1];
    ++servo_pos_now[3][1];
    delay(30);
  }
  flag = 6;
}

void move_FristJoint(int pos1, int pos2, int pos3, int pos4) {
  servo[0][0].write(pos1);
  servo[1][0].write(pos2);
  servo[2][0].write(pos3);
  servo[3][0].write(pos4);
}

void move_SecondJoint(int pos1, int pos2, int pos3, int pos4) {
  servo[0][1].write(pos1);
  servo[1][1].write(pos2);
  servo[2][1].write(pos3);
  servo[3][1].write(pos4);
}

void move_ThirdJoint(int pos1, int pos2, int pos3, int pos4) {
  servo[0][2].write(pos1);
  servo[1][2].write(pos2);
  servo[2][2].write(pos3);
  servo[3][2].write(pos4);
}

void dataReset() {
  data.Joy_X = 510;
  data.Joy_Y = 530;
  data.Joy_SW = 1;
  data.SW_A = 1;
  data.SW_B = 1;
  data.SW_C = 1;
  data.SW_D = 1;
  data.Toggle1_SW = 1;
  data.Toggle2_SW = 1;
  data.Pot1 = 1023;
}
