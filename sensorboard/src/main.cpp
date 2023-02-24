#include "header.h"

volatile float powerLevel = 0.f;

void ROSCoreCommunication_task(void* parameters){
  while(true){
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void measureBatteryState_task(void* parameters){
  while(true){
    powerLevel = (float)analogRead(PWR_IN) / 4095.f;
    if(powerLevel < 0.88)
      digitalWrite(PWR_LED, HIGH);
    else
      digitalWrite(PWR_LED, LOW);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(115200);

  pinMode(PWR_IN, INPUT);
  pinMode(PWR_LED, OUTPUT);
  pinMode(MTC, OUTPUT);

  xTaskCreate(
    ROSCoreCommunication_task,
    "Task PWR",
    1000,
    NULL,
    2,
    NULL
  );

  xTaskCreate(
    measureBatteryState_task,
    "measureBatteryState_task",
    500,
    NULL,
    1,
    NULL
  );
}

void loop() {

  // Serial.print(analogRead(PWR_IN));
  // Serial.print(" ");
  // Serial.println(powerLevel);
  digitalWrite(MTC, HIGH);

  Serial.write(Serial2.read());
  
  vTaskDelay(10 / portTICK_PERIOD_MS);
}