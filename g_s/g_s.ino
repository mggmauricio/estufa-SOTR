#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>
#include <task.h>
#include <semphr.h>

SoftwareSerial hc12(10, 11);
// SemaphoreHandle_t semaforoEnvioPWM;
int pwmReceived;
int temperaturaAtual;  // Variável global para armazenar a temperatura medida

void readTemperatureTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    temperaturaAtual = analogRead(A0);
    // Serial.println(temperaturaAtual);
    // Envie a temperatura via rádio HC-12
    hc12.write(temperaturaAtual);

    // Atraso de 500ms (0.5 segundo)
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void sendPWMTask(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    // Serial.print("tanana");
    // Tente receber um valor de PWM do rádio HC-12
    if (hc12.available() > 0){
      Serial.println("tanana");
      pwmReceived = hc12.read();
      Serial.write(pwmReceived);
      // Envie o valor de PWM para a porta 3
      analogWrite(3, pwmReceived);
    }
    // Serial.println(pwmReceived);
    // Atraso de 1000ms (1 segundo)
    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}

void setup() {
  Serial.begin(9600);
  hc12.begin(9600);

  // semaforoEnvioPWM = xSemaphoreCreateBinary();

  xTaskCreate(readTemperatureTask, "ReadTemp", 100, NULL, configMAX_PRIORITIES - 1, NULL);
  xTaskCreate(sendPWMTask, "SendPWM", 100, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Este loop não deve ser utilizado com FreeRTOS,
  // todas as tarefas são gerenciadas pelo sistema operacional
}
