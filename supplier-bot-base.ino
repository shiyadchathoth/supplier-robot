#define FORWARD_C 'A'
#define BACKWARD_C 'B'
#define LEFT_C 'D'
#define RIGHT_C 'E'
#define STOP_C 'C'

#define GOTO_KITCHEN 'K'
#define GOTO_TABLE 'T'

#define trigPin 9
#define echoPin1 2
#define echoPin2 3
 

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

MotorPin motorL = { 5, 6 };
MotorPin motorR = { 7, 8 };
IRSensorPin irSensor = { A3, A2, A0, A1 };
bool prevSensorVal[4];
bool sensorVal[4];

char mode = 'm';
char tmpMode;

unsigned long lastCollissionMsgTime = 0;
unsigned long lastCollissionCheck = 0;

bool collissionDetected = false; 

volatile bool echoState1, echoState2;
volatile unsigned long timerStart1, timerStart2, pulseDuration1, pulseDuration2;

char moveDir = 's';

int distance1, distance2;

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

void isr1() {
  echoState1 = digitalRead(echoPin1);
   if (echoState1) {
    timerStart1 = micros();
    return;
  }
  pulseDuration1 = micros() - timerStart1;
}

void isr2() {
  echoState2 = digitalRead(echoPin2);
   if (echoState2) {
    timerStart2 = micros();
    return;
  }
  pulseDuration2 = micros() - timerStart2;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(motorL.pin1, OUTPUT);
  pinMode(motorL.pin2, OUTPUT);
  pinMode(motorR.pin1, OUTPUT);
  pinMode(motorR.pin2, OUTPUT);

  pinMode(irSensor.frontL, INPUT);
  pinMode(irSensor.frontR, INPUT);
  pinMode(irSensor.backL, INPUT);
  pinMode(irSensor.backR, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);

  pinMode(body.tablePoint, INPUT_PULLUP);
  pinMode(body.kitchenPoint, INPUT_PULLUP);
  pinMode(body.motorPin1, OUTPUT);
  pinMode(body.motorPin2, OUTPUT);
  

  attachInterrupt(digitalPinToInterrupt(echoPin1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin2), isr2, CHANGE);
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
    case LEFT_C: setMotorPins(LOW, LOW, HIGH, LOW);break;
    case RIGHT_C: setMotorPins(HIGH, LOW, LOW, LOW);break;
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
  }

  sensorVal[3] = !digitalRead(irSensor.frontL);
  sensorVal[2] = !digitalRead(irSensor.frontR);
  sensorVal[1] = !digitalRead(irSensor.backL);
  sensorVal[0] = !digitalRead(irSensor.backR);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delay(50);

  distance1 = pulseDuration1 * 0.034 / 2;
  distance2 = pulseDuration2 * 0.034 / 2;
  collissionDetected = distance1 < 20 || distance2 < 20;
  
  if (mode == 'k') kitchenMode();
  else if (mode == 't' && !collissionDetected) tableMode();
  else if (mode != 'm' && collissionDetected) {
    setMotorPins(LOW, LOW, LOW, LOW);
    if (lastCollissionMsgTime == 0 || millis() - lastCollissionMsgTime > 2000) {
        Serial.print('O');
        lastCollissionMsgTime = millis();
      }
  } else if (mode == 'm' && collissionDetected && moveDir == FORWARD_C) {
      setMotorPins(LOW, LOW, LOW, LOW);
      if (lastCollissionMsgTime == 0 || millis() - lastCollissionMsgTime > 2000) {
        Serial.print('O');
        lastCollissionMsgTime = millis();
      }
  }


  for(int i = 0; i < 4; i++) prevSensorVal[i] = sensorVal[i];

  updateBodyPos(body);
  moveBody(body, body.dir);
}