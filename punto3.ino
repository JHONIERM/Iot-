#include <EEPROM.h>
//#include <Task.h>
//#include <WrapperFreeRTOS.h>

#include <EmbeddedMqttBroker.h>

//#include <Arduino_FreeRTOS.h>
//#include <FreeRTOS.h>
#include <dummy.h>
//#include <Arduino.h>
#include <task.h>
#include <Preferences.h>



// Rest of your existing code

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
bool showTask1 = true;
unsigned long buttonPressStartTime = 0;
bool buttonPressed = false;
bool deepSleepRequested = false;

const int BUTTON_PIN = 0; // Actualiza esto con el pin real del botón
const unsigned long TIME_TO_SLEEP = 10; // Tiempo de sueño en segundos

Preferences preferences; // Objeto para manejar la memoria NVS

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

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  while (1) {
    if (!showTask1) {
      buttonState = digitalRead(BUTTON_PIN);
      //Serial.print("Button State: ");
      //Serial.println(buttonState);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    buttonState = digitalRead(BUTTON_PIN);

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
  delay(1000); // Añade un retraso para permitir que la comunicación serial se estabilice

  // Inicializa la memoria NVS
  preferences.begin("myApp", false);

  // Restaura el último estado desde la memoria NVS
  int lastState = preferences.getBool("lastState", true);
  showTask1 = lastState;

  xTaskCreate(Task1, "Task1", 10000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Task2", 10000, NULL, 1, &Task2Handle);

}

void loop() {
if (digitalRead(BUTTON_PIN) == LOW) {
    if (!buttonPressed) {
      buttonPressStartTime = millis();
      buttonPressed = true;
    } else {
      unsigned long currentTime = millis();
      unsigned long buttonPressDuration = currentTime - buttonPressStartTime;
      if (buttonPressDuration >= 10000) {
        // Button pressed for 10 seconds, request deep sleep
        deepSleepRequested = true;
        buttonPressed = false;

        // Guarda el estado actual en la memoria NVS antes de entrar en el sueño
        preferences.putBool("lastState", showTask1);
        preferences.end(); // Finaliza el uso de la memoria NVS
      }
    }
  } else {
    buttonPressed = false;
  }
  // Realiza el modo de deep sleep si se solicita
  if (deepSleepRequested) {
    Serial.println("Entrando en modo de deep sleep...");
    // Agrega cualquier limpieza necesaria o guardado de estado aquí antes del sueño
    esp_deep_sleep(1000000ULL * TIME_TO_SLEEP);  // Convierte segundos a microsegundos
  }
}
