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


void Task1(void *pvParameters) {
  while (1) {
    int voltage = analogRead(A0);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Pausa de 1000 ms
  }
}

void Task2(void *pvParameters) {
  while (1) {
    int buttonState = digitalRead(0); // Cambia el número de pin según corresponda
    Serial.print("Button State: ");
    Serial.println(buttonState);
    vTaskDelay(pdMS_TO_TICKS(50)); // Pausa de 50 ms
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT); // Configura el pin del botón como entrada
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, &Task1Handle, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, &Task2Handle, 1);
}

void loop() {
  // Nada que hacer aquí, las tareas se encargan del trabajo
}
