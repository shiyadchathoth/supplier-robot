#include <Servo.h>
#define FORWARD_C 'A'
#define BACKWARD_C 'B'
#define LEFT_C 'D'
#define RIGHT_C 'E'
#define STOP_C 'C'


#define GOTO_KITCHEN 'K'
#define GOTO_TABLE 'T'
Servo myservo;

struct MotorPin {
  int pin1;
  int pin2;
};

struct IRSensorPin {
  int frontL;
  int frontR;
  int backL;
  int backR;
};

MotorPin motorL = { 3, 4 };
MotorPin motorR = { 5, 6 };
IRSensorPin irSensor = { A3, A2, A0, A1 };
bool prevSensorVal[4];
bool sensorVal[4];

char mode = 'm';
char tmpMode;

bool collissionDetected = false;

char moveDir = 's';

struct BodyState {
  int tablePoint; //10
  int kitchenPoint;// 11
  int pos;
  int dir;
  int motorPin1;
  int motorPin2;
};

BodyState body = BodyState { 10, 11, 0, 0, A4, A5 };

void moveBody(BodyState &body, int dir) {
  digitalWrite(body.motorPin1, dir == -1);
  digitalWrite(body.motorPin2, dir == 1);
}

void moveToTable(BodyState &body) {
  if (body.dir != 0) return;
  body.dir = 1;
}

void moveToKitchen(BodyState &body) {
  if (body.dir != 0) return;
  body.dir = -1;
}

void updateBodyPos(BodyState &body) {
  if (!digitalRead(body.tablePoint)) body.pos = 1;
  else if (!digitalRead(body.kitchenPoint)) body.pos = -1;
  else body.pos = 0;

  if (body.dir == -1 && body.pos == -1) body.dir = 0; //stop moving if the limit has reached
  if (body.dir == 1 && body.pos == 1) body.dir = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(0);
  pinMode(motorL.pin1, OUTPUT);
  pinMode(motorL.pin2, OUTPUT);
  pinMode(motorR.pin1, OUTPUT);
  pinMode(motorR.pin2, OUTPUT);

  pinMode(irSensor.frontL, INPUT);
  pinMode(irSensor.frontR, INPUT);
  pinMode(irSensor.backL, INPUT);
  pinMode(irSensor.backR, INPUT);

  pinMode(body.tablePoint, INPUT_PULLUP);
  pinMode(body.kitchenPoint, INPUT_PULLUP);
  pinMode(body.motorPin1, OUTPUT);
  pinMode(body.motorPin2, OUTPUT);
}

void setMotorPins(bool mLp1, bool mLp2, bool mRp1, bool mRp2) {
  digitalWrite(motorL.pin1, mLp1);
  digitalWrite(motorL.pin2, mLp2);
  digitalWrite(motorR.pin1, mRp1);
  digitalWrite(motorR.pin2, mRp2);
}

//dir: -1, 0, 1 : left, forward/backward, right
void moveA(char dir, bool fwd = true) {
  switch (dir) {
    case 'f': setMotorPins(fwd, !fwd, fwd, !fwd); break;
    case 'r': setMotorPins(fwd, !fwd, !fwd, !fwd); break;
    case 'l': setMotorPins(!fwd, !fwd, fwd, !fwd); break;
    case 's': setMotorPins(LOW, LOW, LOW, LOW);
  }
}

void kitchenMode() {
  if (sensorVal[3] && sensorVal[2]) {
    moveA('f', true);
  } else if (!sensorVal[3]&& !sensorVal[2]) {
    moveA('s', true);
    if (prevSensorVal[3] == sensorVal[3] && prevSensorVal[2] == sensorVal[2]) return;
    Serial.print('W');
    delay(2000);
    Serial.print('W');
    delay(2000);
    Serial.print('W');
    delay(2000);
  } else if (sensorVal[3] && !sensorVal[2]) {
    moveA('r', true);
  } else if (!sensorVal[3] && sensorVal[2]) {
    moveA('l', true);
  }
} 

void tableMode() {
  if (sensorVal[1] && sensorVal[0]) {
     moveA('f', false);
  } else if (!sensorVal[1] && !sensorVal[0]) {
    moveA('s', false);
    if (prevSensorVal[1] == sensorVal[1] && prevSensorVal[0] == sensorVal[0]) return;
    Serial.print('Z');
    delay(2000);
    Serial.print('Z');
    delay(2000);
    Serial.print('Z');
    delay(2000);
  } else if (sensorVal[1] && !sensorVal[0]) {
     moveA('r', false);
  } else if (!sensorVal[1] && sensorVal[0]) {
     moveA('l', false);
  }
}

void moveManual(char dir) {
  moveDir = dir;
  tmpMode = mode;
  mode = 'm';
  switch (dir) {
    case FORWARD_C: setMotorPins(HIGH, LOW, HIGH, LOW);
      moveToKitchen(body);
      break;
    case BACKWARD_C: setMotorPins(LOW, HIGH, LOW, HIGH);
      moveToTable(body);
      break;
    case LEFT_C: setMotorPins(LOW, LOW, HIGH, LOW); break;
    case RIGHT_C: setMotorPins(HIGH, LOW, LOW, LOW); break;
    case STOP_C: setMotorPins(LOW, LOW, LOW, LOW); break;
    default:  mode = tmpMode;
  }
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    moveManual(c);
    if (c == GOTO_KITCHEN) {
      mode = 'k';
      moveToKitchen(body);
    }
    if (c == GOTO_TABLE) {
      mode = 't';
      moveToTable(body);
    }
        if (c == 'W') {
    myservo.write(30);
    delay(100); 
    myservo.write(0);
    }
      if (c == 'R') {
    myservo.write(0);
    
    }
  }

  sensorVal[3] = !digitalRead(irSensor.frontL);
  sensorVal[2] = !digitalRead(irSensor.frontR);
  sensorVal[1] = !digitalRead(irSensor.backL);
  sensorVal[0] = !digitalRead(irSensor.backR);

  if (mode == 'k') kitchenMode();
  else if (mode == 't' && !collissionDetected) tableMode();
  else if (mode != 'm' && collissionDetected) {
    setMotorPins(LOW, LOW, LOW, LOW);
    // Collision handling logic
  } else if (mode == 'm' && collissionDetected && moveDir == FORWARD_C) {
    setMotorPins(LOW, LOW, LOW, LOW);
    // Collision handling logic for manual forward movement
  }

  for (int i = 0; i < 4; i++) prevSensorVal[i] = sensorVal[i];

  updateBodyPos(body);
  moveBody(body, body.dir);
}
