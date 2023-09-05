//#include <Task.h>
//#include <WrapperFreeRTOS.h>

#include <EmbeddedMqttBroker.h>

//#include <Arduino_FreeRTOS.h>
//#include <FreeRTOS.h>
#include <dummy.h>
//#include <Arduino.h>
#include <task.h>

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
bool showTask1 = true;
unsigned long buttonPressStartTime = 0;
bool buttonPressed = false;


void Task1(void *pvParameters) {
  while (1) {
    if (showTask1) {
      int voltage = analogRead(A0);
      Serial.print("Voltage: ");
      Serial.println(voltage);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Task2(void *pvParameters) {
  int buttonState = HIGH;
  int lastButtonState = HIGH;
  bool buttonPressed = false;

  pinMode(0, INPUT_PULLUP);

  while (1) {
    if (!showTask1) {
      buttonState = digitalRead(0);
      //Serial.print("Button State: ");
      //Serial.println(buttonState);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    buttonState = digitalRead(0);

    if (buttonState != lastButtonState) {
      if (buttonState == LOW && !buttonPressed) {
        buttonPressed = true;

        if (showTask1) {
          vTaskSuspend(Task1Handle);
          showTask1 = false;
          Serial.println("Task 1 suspended");
        } else {
          vTaskResume(Task1Handle);
          showTask1 = true;
          Serial.println("Task 1 resumed");
        }
      }
      
    } else {
      buttonPressed = false;
    }
    

    lastButtonState = buttonState;
  }
}

void setup() {
  Serial.begin(115200);
  xTaskCreate(Task1, "Task1", 10000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Task2", 10000, NULL, 1, &Task2Handle);
}

void loop() {
  // Nada que hacer aquí, las tareas se encargan del trabajo
}
