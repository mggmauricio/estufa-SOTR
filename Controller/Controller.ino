#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>
#include <semphr.h>
// Inclua aqui as bibliotecas específicas para o seu hardware, como para o sensor, controlador PID, PWM e display.

// Defina os pinos para o sensor, PWM e display.
#define SENSOR_PIN A0
#define PWM_PIN 3
#define UP_PIN 13
#define LOW_PIN 12
#define SETPOINT A1


// Defina as variáveis globais necessárias para o seu aplicativo.
float sensorValue = 0.0;

#define QUEUE_SIZE 5
QueueHandle_t sensorQueue;

float x1 = 0.0;
float b0 = 0.10243; 
float b1 = 0.005848753; 
float b2 = -0.096581247;
float a1 = 0.94186, a2 = -0.0574;
int setpoint = 20;
float y1 = 0.0;
static float ek1 = 0, ek2 = 0;
static float uk1 = 0, uk2 = 0;
int pwm;

// Protótipos de funções para as tarefas.
void taskReadSensor(void *pvParameters);
void taskController(void *pvParameters);
void taskPWM(void *pvParameters);
void taskSetpoint(void *pvParameters);
void taskUpdateDisplay(void *pvParameters);

SemaphoreHandle_t xSemaphore; 
SemaphoreHandle_t xSetpointMutex;

void setup() {
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(LOW_PIN, INPUT_PULLUP);
  pinMode(A1, INPUT);    
  pinMode(SENSOR_PIN, INPUT);
  Serial.begin(9600);

  sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));

  digitalWrite(2, HIGH);
  // Crie as tarefas.
  xTaskCreate(taskReadSensor, "ReadSensor", 128, NULL, 2, NULL);
  xTaskCreate(taskController, "Controller", 128, NULL, 2, NULL);
  xTaskCreate(taskSetpoint, "Setpoint", 128, NULL, 2, NULL);

  xSemaphore = xSemaphoreCreateBinary(); // Inicialize o semáforo
  xSetpointMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {
  // analogWrite(PWM_PIN, 180);
  // O loop não será executado, pois o FreeRTOS cuidará das tarefas.
}






void taskReadSensor(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    int reading = analogRead(SENSOR_PIN);
    sensorValue = (float(analogRead(SENSOR_PIN))*5/(1023))/0.01;
    if(uxQueueSpacesAvailable(sensorQueue) == 0) {
      // A fila está cheia. Remova a leitura mais antiga.
      xQueueReceive(sensorQueue, NULL, 0);
    }
    // Adicione a nova leitura à fila.
    xQueueSend(sensorQueue, &sensorValue, portMAX_DELAY);
    xSemaphoreGive(xSemaphore); // Libere o semáforo após a leitura do sensor
    vTaskDelay(pdMS_TO_TICKS(60));
  } 
}

void calculateControl() {
  float ek0 = (float)setpoint - sensorValue;
  Serial.print("Sensor: ");
  Serial.println(sensorValue);
  float uk0 = a1 * uk1 + a2 * uk2 + b0 * ek0 + b1 * ek1 + b2 * ek2;
  uk2 = uk1;
  uk1 = uk0;
  ek2 = ek1;
  ek1 = ek0;
  y1 = uk0;

  pwm = map(y1 * 100, -7, 18, 0, 255);
  if (pwm >= 255) {
    pwm = 255;
  } else if (pwm <= 0) {
    pwm = 0;
  }
  analogWrite(PWM_PIN, pwm);
}

void taskController(void *pvParameters) {
  (void)pvParameters;

  float receivedValue;

  for (;;) {
    // Aguarde até que um valor seja recebido na fila.
    if (xQueueReceive(sensorQueue, &receivedValue, portMAX_DELAY)) {
      xSemaphoreTake(xSemaphore, portMAX_DELAY); // Bloqueie o acesso a sensorValue

      // Chame a função de cálculos
      calculateControl();

      xSemaphoreGive(xSemaphore);  // Libere o acesso a sensorValue
    }

    // Aguarde um curto período antes de calcular novamente.
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskSetpoint(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    // Realize a leitura do valor do setpoint a partir do pino analógico A1.
    int analogValue = analogRead(A1);
    xSemaphoreTake(xSetpointMutex, portMAX_DELAY);
    setpoint = map(analogValue, 0, 1020, 20, 80);
    xSemaphoreGive(xSetpointMutex); // Libere o acesso ao setpoint

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}