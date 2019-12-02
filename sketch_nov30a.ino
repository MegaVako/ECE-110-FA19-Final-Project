#include "IRremote.h"
#include "NewPing.h"
#include <Servo.h>

const int pinArr[]  = {4, 7, 3, 2}; //LF, LB, RF, RB 
const int speedpinArr[] = {6, 5};
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2

boolean goesForward = false;
int distance = 100;
int prevDistance = 100;
const int allowedDistance = 300;
const int allowChange = allowedDistance * 0.3;
int servo_pin = 8;
int servoInit = 90;
int signal_pin = 13;

int PWM_LR_diff = 0;
int PWM_control = 0;
const int PWM_rate = 15;
const int PWM_max = 255;
const int PWM_min = 0;
const int PWM_init = 60;
bool LED_display_PWM = true; // or distance;
bool LED_on = true;

const int LEDArr[] = {9, 10, 11}; // Green, Yello, Red

const int RECV_PIN = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;
bool continuous_move = false;

Servo servo_motor;

enum state {
  pause,
  freeMovement,
  adv_freeMovement,
  stop_state,
  force_foward,
  force_backward,
  force_turnL,
  force_turnR,
  follow,
  testPing,
};
state carState = freeMovement;
long inputSignal = 0x0;
NewPing sonar(trig_pin, echo_pin, allowedDistance);

int readPing() {
  int res = sonar.ping_cm();
  if (res == 0) return allowedDistance;
  return sonar.ping_cm();
}



void setup(){
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);

  //sonic
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  initDistance();
  //motor
  for (int i = 0; i < 4; i++) {
    pinMode(pinArr[i], OUTPUT);
    if (i < 2) pinMode(speedpinArr[i], OUTPUT);
    if (i < 3) pinMode(LEDArr[i], OUTPUT);
  }
  //servo
  servo_motor.attach(servo_pin);
  PWM_control = PWM_init;
  carState = stop_state;
  inputSignal = 0x0;
  LED_controller();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void loop(){
  //Serial.println(distance);
  if (irrecv.decode(&results)){
        Serial.println("signal received");
        Serial.println(results.value, HEX);
        inputSignal = results.value;
        irrecv.resume();
        signalDecoder();
        analogWrite(signal_pin, 255);
        delay(200);
        analogWrite(signal_pin, 0);
  }
  switch(carState) {
    case freeMovement:
      state_freeMovement();
      break;
    case force_foward:
      foward();
      break;
    case force_backward:
      backward();
      break;
    case force_turnL:
      turn(true); // bool left
      break;
    case force_turnR:
      turn(false);
      break;
    case stop_state:
    case pause:
      stay();
      break;
    case follow:
      state_follow();
      break;
    case testPing:
      stay();
      delay(100);
      Serial.println(readPing());
      break;
    case adv_freeMovement:
      state_adv_freeMovement();
      break;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initDistance() {
  servo_motor.write(servoInit); // ?
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  prevDistance = distance;
}
void stay() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pinArr[i], LOW);
    if (i < 2) analogWrite(speedpinArr[i], PWM_min);
  }
}

void movement(bool foward) {
  goesForward = foward;
  int dir = !foward;
  for (int i = 0; i < 4; i++) { // dir == 0 for forward
    if (i % 2 == dir) digitalWrite(pinArr[i], HIGH);
    else digitalWrite(pinArr[i], LOW);
  }
  if (!continuous_move) {
    analogWrite(speedpinArr[0], PWM_control);
    analogWrite(speedpinArr[1], PWM_control);
  } else {
    analogWrite(speedpinArr[0], PWM_control - PWM_LR_diff);
    analogWrite(speedpinArr[1], PWM_control + PWM_LR_diff);
  }
}

void turn(bool left) { // if right h l l h
  // if left l h h l
  if (!continuous_move) {
    int dir = left;
    for (int i = 0; i < 4; i++) { // dir == 0 for forward
      if (!left) {
        if (i == 0 || i == 3) digitalWrite(pinArr[i], HIGH);
        else digitalWrite(pinArr[i], LOW);
      } else {
        if (i == 0 || i == 3) digitalWrite(pinArr[i], LOW);
        else digitalWrite(pinArr[i], HIGH);
      }
      if (i < 2) analogWrite(speedpinArr[i], PWM_control);
    }
  } else {
    movement(goesForward);
  }
}
void foward() {
  movement(true);
}
void backward() {
  movement(false);
}

int Ldir = 170;
int Rdir = 10;
int look(bool L) {
  Serial.println("looking around");
  int dir = -1;
  if (L) dir = Ldir;
  else dir = Rdir;
  servo_motor.write(dir);
  delay(500);
  int reading = readPing();
  delay(100);
  servo_motor.write(servoInit);
  return reading;
}

int delay1 = 50; // in ms
int stopDelayInterval = 500;
int stopDistance = 30;
void state_freeMovement() {
  distance = readPing();
  delay(delay1);
  if (distance < stopDistance) {
    tooCloseTurn();
  } else {
    foward();
  }
}
void tooCloseTurn() {
  initPWMLR();
  continuous_move = false;
  int dL = 0;
  int dR = 0;
  stay();
  delay(stopDelayInterval);
  backward();
  delay(500);
  stay();
  delay(stopDelayInterval);
  dL = look(true); // L bool
  delay(stopDelayInterval);
  dR = look(false);
  delay(stopDelayInterval); // stay until now
  if (dL < dR) {
    turn(false); //bool left
  } else {
    turn(true); //bool left
  }
  delay(500);
  stay();
  delay(stopDelayInterval);
  foward();
}
int stopDistanceCap = 30;
int tolerance = 15;
void state_follow() {
  distance = readPing();
  Serial.println(distance);
  if (distance > stopDistanceCap + tolerance) {
    foward();
  } else if (distance < stopDistanceCap - tolerance) {
    backward();
  } else {
    stay();
  }
  delay(delay1 * 2);
}
state prevState = pause;
void signalDecoder() {
  Serial.println("decoder==================");
  irrecv.enableIRIn();
  switch(inputSignal) {
    case 0xFD08F7: //key 1 
      carState = freeMovement;
      servo_motor.write(servoInit);
      break;
    case 0xFD30CF: //key 0
      carState = stop_state;
      servo_motor.write(servoInit);
      break;
    case 0xFDA05F: // >||
      if (carState == pause) carState = prevState;
      else {
        prevState = carState;
        carState = pause;
      }
      servo_motor.write(servoInit);
      break;
    case 0xFD807F: //vol+
      carState = force_foward;
      break;    
    case 0xFD906F: //vol-
      carState = force_backward;
      break;
    case 0xFD20DF: //|<<
      carState = force_turnL;
      if (continuous_move) turnPWMLR(true);
      break;
    case 0xFD609F:
      carState = force_turnR;
      if (continuous_move) turnPWMLR(false);
      break;
    case 0xFD8877: //key 2
      reset();
      carState = follow;
      servo_motor.write(servoInit);
      break;
    case 0xFD50AF: //up arrow
      if (PWM_control < PWM_max) PWM_control += PWM_rate;
      LED_controller();
      Serial.println(PWM_control);
      break;
    case 0xFD10EF: //down arrow
      if (PWM_control > PWM_min) PWM_control -= PWM_rate;
      LED_controller();
      Serial.println(PWM_control);
      break;
    case 0xFD58A7: //key 9
      carState = testPing;
      LED_display_PWM = false;
      break;
    case 0xFD48B7: //key 3
      continuous_move = !continuous_move;
      stay();
      initPWMLR();
      delay(1000);
      break;
    case 0xFD40BF: //func/stop key 
      resetPWMLR();
      break;
    case 0xFDB04F: //key EQ
      reset();
      break;
    case 0xFD708F: //key ST/REPT
      LED_display_PWM = !LED_display_PWM;
      LED_controller();
      break;
    case 0xFD00FF: //power key
      LED_on = !LED_on;
      LED_controller();
      break;
    case 0xFDA857: //key 5
      initDistanceScan();
      carState = adv_freeMovement;
      break;
  }
}
void initPWMLR() {
  PWM_LR_diff = 0;
}
int PWM_moving_min = 50;
void resetPWMLR() { 
  PWM_LR_diff = 0;
  Serial.println("====pwm diff reset==========");
}
int PWM_dir_change_minor = 5;
void turnPWMLR(bool left) { // positive diff for turn left
  if (left) {
    if (PWM_control + PWM_LR_diff + PWM_dir_change_minor < PWM_max) PWM_LR_diff += PWM_dir_change_minor;
  } else {
    if (PWM_control - PWM_LR_diff - PWM_dir_change_minor > PWM_min) PWM_LR_diff -= PWM_dir_change_minor;
  }
  Serial.println(PWM_LR_diff);
  Serial.println("====diff==========");
}
void reset() {
  PWM_control = PWM_init;
  PWM_LR_diff = 0;
  carState = stop_state;
  inputSignal = 0x0;
  continuous_move = false;
  LED_display_PWM = true;
  LED_controller();
}
int lightFactor_PWM = (PWM_max / 3);
int lightFactor_distance = allowedDistance / 3;
int LED_Von = 130;
int light_range = 255 - LED_Von;
void LED_controller() {
  Serial.println("LED");
  for (int i = 0; i < 3; i++) {
    analogWrite(LEDArr[i], 0);
  }
  if (LED_on) {
    double currInput = 0;
    double currFactor = 0;
    if (LED_display_PWM) {
      currInput = PWM_control;
      currFactor = lightFactor_PWM;
    } else {
      currInput = distance;
      currFactor = lightFactor_distance;
    }
    bool shouldLightMore = false;
    Serial.println(1);
    if (currInput < currFactor) {
      double temp = currInput / currFactor;
      double curr1 = temp * light_range + LED_Von;
      Serial.println("curr1 = " + String(curr1));
      Serial.println(temp * light_range);
      if (curr1 > LED_Von) analogWrite(LEDArr[0], curr1);
    }
    else {
      analogWrite(LEDArr[0], 255);
      shouldLightMore = true;
      Serial.println(2);
    }
    if (shouldLightMore) {
      if (currInput < currFactor * 2) {
        float curr2 = (currInput - currFactor) / currFactor * light_range + LED_Von;
        Serial.println("curr2 = " + String(curr2));
        if (curr2 > LED_Von) analogWrite(LEDArr[1], curr2);
        shouldLightMore = false;
      } else {
        analogWrite(LEDArr[1], 255);
        Serial.println(3);
      }
    }
    if (shouldLightMore) {
      float curr3 = (currInput - currFactor * 2) / currFactor * light_range + LED_Von;
      if (curr3 > LED_Von) {
        analogWrite(LEDArr[2], curr3);
        Serial.println("curr3 = " + String(curr3));
      }
    }
    irrecv.enableIRIn(); // Start the receiver
    delay(10);
  }
}

int startScanDeg = servoInit - 40;
int endScanDeg = servoInit + 40;
int scanCounter = startScanDeg;
bool scanTurnR = true;
int scanInterval = 10;
void distanceScan() {
  servo_motor.write(scanCounter);
  int d1 = readPing();
  delay(40);
  int d2 = readPing();
  int diff = d2 - d1;
  Serial.println("====================");
  Serial.println(scanCounter);  
  Serial.println(diff);
  Serial.println(d2);
  Serial.println("====================");
  if (scanCounter == 90 && d2 <= stopDistance) { 
    tooCloseTurn();
    initDistanceScan();
  } else {
    if (diff < 0 && d2 <= stopDistance) {
      int prevMeas = d2;
      int turnCounter = 0;
      while (diff < 0 && d2 <= stopDistance) {
        ++turnCounter;
        turn(!(scanCounter < servoInit)); // bool left // if in left turn right
        delay(300);
        if (turnCounter > 6) {
          tooCloseTurn();
          initDistanceScan();
          break;
        }
        d2 = readPing();
        diff = prevMeas - d2;
        prevMeas = d2;
      }
      initPWMLR();
    }
    
    if (scanTurnR) scanCounter += scanInterval;
    else scanCounter -= scanInterval;
    
    if (scanCounter >= endScanDeg + 1) {
      scanTurnR = false;
    } else if (scanCounter <= startScanDeg - 1) {
      scanTurnR = true;
    }
  }
}
void initDistanceScan() {
  scanCounter = startScanDeg;
  scanTurnR = true;
}
void state_adv_freeMovement() { // need to add to decode
  distanceScan();
  delay(10);
  foward();
  continuous_move = true;
}
