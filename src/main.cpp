#include "header.h"

// B pin of encoder is not used. The direction of the motors will be deduced from H-Bridge. This is, however, a precision flaw
volatile uint16_t count_BL = 0;
volatile uint16_t count_BR = 0;
volatile uint16_t count_FL = 0;
volatile uint16_t count_FR = 0;

double rotationspeed_BL = 0;
double rotationspeed_BR = 0;
double rotationspeed_FL = 0;
double rotationspeed_FR = 0;

double dutyCycle_BL = 60;
double dutyCycle_BR = 60;
double dutyCycle_FL = 60;
double dutyCycle_FR = 60;

double wantedWheelVel_BL = 0;
double wantedWheelVel_BR = 0;
double wantedWheelVel_FL = 0;
double wantedWheelVel_FR = 0;

// Interrup routines
void IRAM_ATTR function_ISR_EC_BL() {
  // Encoder out A triggers interrupt
  // TODO: check last B state to determine direction
  count_BL++;
}

void IRAM_ATTR function_ISR_EC_BR() {
  count_BR++;
}

void IRAM_ATTR function_ISR_EC_FL() {
  count_FL++;
}

void IRAM_ATTR function_ISR_EC_FR() {
  count_FR++;
}

// PID init
double Kp=2, Ki=1, Kd=1;

PID PID_BL(&wantedWheelVel_BL, &dutyCycle_BL, &rotationspeed_BL, Kp, Ki, Kd, AUTOMATIC);
PID PID_BR(&wantedWheelVel_BR, &dutyCycle_BR, &rotationspeed_BR, Kp, Ki, Kd, AUTOMATIC);
PID PID_FL(&wantedWheelVel_FL, &dutyCycle_FL, &rotationspeed_FL, Kp, Ki, Kd, AUTOMATIC);
PID PID_FR(&wantedWheelVel_FR, &dutyCycle_FR, &rotationspeed_FR, Kp, Ki, Kd, AUTOMATIC);

BLA::Matrix<4> calculateWheelVelocity(BLA::Matrix<3> robotVelocity){
  
  BLA::Matrix<4> wheelVelocity;
  BLA::Matrix<4, 3> forwardKinematicsModel = { 1, -1, -(L_X + L_Y),
                                               1, 1, L_X + L_Y,
                                               1, 1, -(L_X + L_Y),
                                               1, -1, L_X + L_Y};
  wheelVelocity = forwardKinematicsModel * robotVelocity;
  wheelVelocity *=  1 / WHEELRADIUS;

  return wheelVelocity;
}

// calculates the velocity in direction x and y, as well as the angular velocity around the z axis
// [m/s] [m/s] [rad/s]
// using the wheel velocities as input
BLA::Matrix<3> calculateRobotVelocity(BLA::Matrix<4> wheelVelocity){
  BLA::Matrix<3> robotVelocity;

  BLA::Matrix<3, 4> inverseKinematicsModel = { 1, 1, 1, 1, 
                                              -1, 1, 1, -1, 
                                              -1/(L_X + L_Y), 1/(L_X + L_Y), -1/(L_X + L_Y), 1/(L_X + L_Y)};

  robotVelocity = inverseKinematicsModel * wheelVelocity;
  robotVelocity *= WHEELRADIUS / 4;

  return robotVelocity;
}

void masterCommunicationRoutine(void* parameters){
  vTaskDelay(80 / portTICK_PERIOD_MS);
}

inline void motorHardwareSetup(){
  ledcSetup(M_BL_PWM_CNL, M_PWM_FRQ, M_PWM_RES);
  ledcSetup(M_BR_PWM_CNL, M_PWM_FRQ, M_PWM_RES);
  ledcSetup(M_FL_PWM_CNL, M_PWM_FRQ, M_PWM_RES);
  ledcSetup(M_FR_PWM_CNL, M_PWM_FRQ, M_PWM_RES);

  ledcAttachPin(M_BL_PWM, M_BL_PWM_CNL);
  ledcAttachPin(M_BR_PWM, M_BR_PWM_CNL);
  ledcAttachPin(M_FL_PWM, M_FL_PWM_CNL);
  ledcAttachPin(M_FR_PWM, M_FR_PWM_CNL);

  pinMode(M_BL_CW, OUTPUT);
  pinMode(M_BL_CCW, OUTPUT);

  pinMode(M_BR_CW, OUTPUT);
  pinMode(M_BR_CCW, OUTPUT);

  pinMode(M_FL_CW, OUTPUT);
  pinMode(M_FL_CCW, OUTPUT);

  pinMode(M_FR_CW, OUTPUT);
  pinMode(M_FR_CCW, OUTPUT);

  //-----Encoder-----

  pinMode(EC_BL_A, INPUT); // hardware pullup
  pinMode(EC_BL_B, INPUT); // hardware pullup
  
  pinMode(EC_BR_A, INPUT); // hardware pullup
  pinMode(EC_BR_B, INPUT); // hardware pullup
  
  pinMode(EC_FL_A, INPUT_PULLUP);
  pinMode(EC_FL_B, INPUT_PULLUP);
  
  pinMode(EC_FR_A, INPUT_PULLUP);
  pinMode(EC_FR_B, INPUT_PULLUP);
  
  attachInterrupt(EC_BL_A, function_ISR_EC_BL, FALLING);
  attachInterrupt(EC_BR_A, function_ISR_EC_BR, FALLING);
  attachInterrupt(EC_FL_A, function_ISR_EC_FL, FALLING);
  attachInterrupt(EC_FR_A, function_ISR_EC_FR, FALLING);
}

void setup() {

  Serial.begin(115200);

  motorHardwareSetup();

  PID_BL.SetMode(AUTOMATIC);
  PID_BR.SetMode(AUTOMATIC);
  PID_FL.SetMode(AUTOMATIC);
  PID_FR.SetMode(AUTOMATIC);
  
  //xTaskCreate(
  //  masterCommunicationRoutine,
  //  "masterCommunicationRoutine",
  //  1000,
  //  NULL,
  //  1,
  //  NULL
  //);
}

unsigned long t = millis();
unsigned long dt = 0;
float PWM1_DutyCycle = 0;
bool TESTCASE = false;

void loop() {
  if(TESTCASE){
    while(PWM1_DutyCycle < 255)
    {
      digitalWrite(M_BL_CW, HIGH);
      digitalWrite(M_BL_CCW, LOW);
      digitalWrite(M_BR_CW, HIGH);
      digitalWrite(M_BR_CCW, LOW);
      digitalWrite(M_FL_CW, LOW);
      digitalWrite(M_FL_CCW, HIGH);
      digitalWrite(M_FR_CW, LOW);
      digitalWrite(M_FR_CCW, HIGH);
      ledcWrite(M_BL_PWM_CNL, PWM1_DutyCycle++);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    while(PWM1_DutyCycle > 0)
    {
      digitalWrite(M_BL_CW, HIGH);
      digitalWrite(M_BL_CCW, LOW);
      digitalWrite(M_BR_CW, HIGH);
      digitalWrite(M_BR_CCW, LOW);
      digitalWrite(M_FL_CW, LOW);
      digitalWrite(M_FL_CCW, HIGH);
      digitalWrite(M_FR_CW, LOW);
      digitalWrite(M_FR_CCW, HIGH);
      ledcWrite(M_BL_PWM_CNL, PWM1_DutyCycle--);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    while(PWM1_DutyCycle < 255)
    {
      digitalWrite(M_BL_CW, LOW);
      digitalWrite(M_BL_CCW, HIGH);
      digitalWrite(M_BR_CW, LOW);
      digitalWrite(M_BR_CCW, HIGH);
      digitalWrite(M_FL_CW, HIGH);
      digitalWrite(M_FL_CCW, LOW);
      digitalWrite(M_FR_CW, HIGH);
      digitalWrite(M_FR_CCW, LOW);
      ledcWrite(M_BL_PWM_CNL, PWM1_DutyCycle++);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    while(PWM1_DutyCycle > 0)
    {
      digitalWrite(M_BL_CW, LOW);
      digitalWrite(M_BL_CCW, HIGH);
      digitalWrite(M_BR_CW, LOW);
      digitalWrite(M_BR_CCW, HIGH);
      digitalWrite(M_FL_CW, HIGH);
      digitalWrite(M_FL_CCW, LOW);
      digitalWrite(M_FR_CW, HIGH);
      digitalWrite(M_FR_CCW, LOW);
      ledcWrite(M_BL_PWM_CNL, PWM1_DutyCycle--);
      vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }else{

    // measure motor speed
    // calculate wanted motor speed with the linear algebra formular above
    // use PID to set the controll accordingly

    dt = millis() - t;
    t = millis();
    if(dt > 0){
      // calculate wanted motor speed from input vector [vel_x, vel_y, rotationspeed] (m/s, m/s, rad/s)
      // TODO: get input vector via rosserial
      BLA::Matrix<4> wantedWheelVel = calculateWheelVelocity({0.0f, 0.0f, 0.0f});
      wantedWheelVel_BL = wantedWheelVel(0);
      wantedWheelVel_BR = wantedWheelVel(1);
      wantedWheelVel_FL = wantedWheelVel(2);
      wantedWheelVel_FR = 1;//wantedWheelVel(3);

      // measure motor speed
      rotationspeed_BL = ((double)count_BL / ((double)dt / (double)1000)) / (double)600;  // rad/s inserted different encoder with 600 BM
      count_BL = 0;
      rotationspeed_BR = ((double)count_BR / ((double)dt / (double)1000)) / (double)360;  // rad/s
      count_BR = 0;
      rotationspeed_FL = ((double)count_FL / ((double)dt / (double)1000)) / (double)360;  // rad/s
      count_FL = 0;
      rotationspeed_FR = ((double)count_FR / ((double)dt / (double)1000)) / (double)360;  // rad/s
      count_FR = 0;

      // compute controllers
      PID_BL.Compute();
      PID_BR.Compute();
      PID_FL.Compute();
      PID_FR.Compute();

      // set rotation speed
      // check direction
      if(wantedWheelVel_BL < 0){
        digitalWrite(M_BL_CW, HIGH);
        digitalWrite(M_BL_CCW, LOW);
      }else if(wantedWheelVel_BL > 0){
        digitalWrite(M_BL_CW, LOW);
        digitalWrite(M_BL_CCW, HIGH);
      }else{
        digitalWrite(M_BL_CW, LOW);
        digitalWrite(M_BL_CCW, LOW);
      }

      if(wantedWheelVel_BR > 0){
        digitalWrite(M_BR_CW, HIGH);
        digitalWrite(M_BR_CCW, LOW);
      }else if(wantedWheelVel_BR < 0){
        digitalWrite(M_BR_CW, LOW);
        digitalWrite(M_BR_CCW, HIGH);
      }else{
        digitalWrite(M_BR_CW, LOW);
        digitalWrite(M_BR_CCW, LOW);
      }

      if(wantedWheelVel_FL < 0){
        digitalWrite(M_FL_CW, HIGH);
        digitalWrite(M_FL_CCW, LOW);
      }else if(wantedWheelVel_FL > 0){
        digitalWrite(M_FL_CW, LOW);
        digitalWrite(M_FL_CCW, HIGH);
      }else{
        digitalWrite(M_FL_CW, LOW);
        digitalWrite(M_FL_CCW, LOW);
      }

      if(wantedWheelVel_FR > 0){
        digitalWrite(M_FR_CW, HIGH);
        digitalWrite(M_FR_CCW, LOW);
      }else if(wantedWheelVel_FR < 0){
        digitalWrite(M_FR_CW, LOW);
        digitalWrite(M_FR_CCW, HIGH);
      }else{
        digitalWrite(M_FR_CW, LOW);
        digitalWrite(M_FR_CCW, LOW);
      }

      ledcWrite(M_BL_PWM_CNL, dutyCycle_BL);//0); //
      ledcWrite(M_BR_PWM_CNL, dutyCycle_BR);//0); //
      ledcWrite(M_FL_PWM_CNL, dutyCycle_FL);//0); //
      ledcWrite(M_FR_PWM_CNL, dutyCycle_FR);//0); //
    }
    // TODO: DutyCicle versucht bei negativer sollgeschw. negativ zu werden... diese blöd
    Serial.print(wantedWheelVel_BL); Serial.print(" "); Serial.print(rotationspeed_BL); Serial.print(" "); Serial.println(dutyCycle_BL);
    Serial.print(wantedWheelVel_BR); Serial.print(" "); Serial.print(rotationspeed_BR); Serial.print(" "); Serial.println(dutyCycle_BR);
    Serial.print(wantedWheelVel_FL); Serial.print(" "); Serial.print(rotationspeed_FL); Serial.print(" "); Serial.println(dutyCycle_FL);
    Serial.print(wantedWheelVel_FR); Serial.print(" "); Serial.print(rotationspeed_FR); Serial.print(" "); Serial.println(dutyCycle_FR);
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}