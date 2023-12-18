//Importacao de bibliotecas
#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>
#include <semphr.h>
#include <timers.h>


// Definicao de pinos
#define SENSOR_PIN A0
#define PWM_PIN 3
#define UP_PIN 13
#define LOW_PIN 12
#define SETPOINT A1

// Definicao de constantes e variáveis
float sensorValue = 0.0;
const float x1 = 0.0;
const float b0 = 0.10243;
const float b1 = 0.005848753;
const float b2 = -0.096581247;
const float a1 = 0.94186, a2 = -0.0574;
int setpoint = 20;
float y1 = 0.0;
static float ek1 = 0, ek2 = 0;
static float uk1 = 0, uk2 = 0;
int pwm;

// Definicao de vetores para guardar os tempos de inicio e fim de tarefas a fim de verificar a funcionalidade do escalonamento
TickType_t startTimes[3];
TickType_t endTimes[3];

// Protótipos de funções para as tarefas.
void taskReadSensor(void *pvParameters);
void taskController(void *pvParameters);
void taskSetpoint(void *pvParameters);


// Timers para as tarefas
TimerHandle_t xReadSensorTimer, xControllerTimer, xSetpointTimer, xSendDataTimer;

// Vetores de periodizacao de tarefas

const TickType_t PERIODS[] = {pdMS_TO_TICKS(200), pdMS_TO_TICKS(150), pdMS_TO_TICKS(100)};
const TickType_t DEADLINES[] = {pdMS_TO_TICKS(50), pdMS_TO_TICKS(70), pdMS_TO_TICKS(100)};


// Semaforos para acesso a variaveis
SemaphoreHandle_t xSemaphore;
SemaphoreHandle_t xSetpointMutex;

// Variáveis para depuração
volatile int debugCounter = 0;
volatile int debugCounterLimit = 1000;  // Ajuste conforme necessário

// Callback de depuração
void debugCallback(TimerHandle_t xTimer) {
  // Serial.println("Debugging...");
  debugCounter++;

  if (debugCounter >= debugCounterLimit) {
    // Serial.println("Debug counter limit reached. Suspending debugging.");
    xTimerStop(xTimer, 0);
  }
}


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  setupTimers();

  xTaskCreate(taskReadSensor, "ReadSensor", 128, NULL, 2, NULL);
  xTaskCreate(taskController, "Controller", 128, NULL, 2, NULL);
  xTaskCreate(taskSetpoint, "Setpoint", 128, NULL, 2, NULL);


  xSemaphore = xSemaphoreCreateBinary();
  xSetpointMutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {
}

void setupTimers() {
  xReadSensorTimer = xTimerCreate("ReadSensorTimer", PERIODS[2], pdTRUE, (void *)0, [](TimerHandle_t xTimer) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
  });

  xControllerTimer = xTimerCreate("ControllerTimer", PERIODS[0], pdTRUE, (void *)0, [](TimerHandle_t xTimer) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
  });

  xSetpointTimer = xTimerCreate("SetpointTimer", PERIODS[1], pdTRUE, (void *)0, [](TimerHandle_t xTimer) {
    xSemaphoreGiveFromISR(xSetpointMutex, NULL);
  });

  xTimerStart(xReadSensorTimer, 0);
  xTimerStart(xControllerTimer, 0);
  xTimerStart(xSetpointTimer, 0);
  // Inicie um timer de depuração para verificar se o sistema está rodando
  TimerHandle_t xDebugTimer = xTimerCreate("DebugTimer", pdMS_TO_TICKS(500), pdTRUE, (void *)0, debugCallback);
  xTimerStart(xDebugTimer, 0);
}



// Task de leitura do sensor que deve se repetir a cada 200ms
void taskReadSensor(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      startTimes[0] = xTaskGetTickCount();
      int reading = analogRead(SENSOR_PIN);
      sensorValue = (float(analogRead(SENSOR_PIN)) * 5 / (1023)) / 0.01;
      endTimes[0] = xTaskGetTickCount();
      sendData(0, startTimes[0], endTimes[0]);
    }

    vTaskDelay(1); // Delay apenas para liberar o processador
  }
}


// Funcao para realizar o calculo do controlador PID
void calculateControl() {
  float ek0 = (float)setpoint - sensorValue;
  float uk0 = a1 * uk1 + a2 * uk2 + b0 * ek0 + b1 * ek1 + b2 * ek2;
  uk2 = uk1;
  uk1 = uk0;
  ek2 = ek1;
  ek1 = ek0;
  y1 = uk0;

  pwm = map(y1 * 100, -7, 30, 0, 255);
  if (pwm >= 255) {
    pwm = 255;
  } else if (pwm <= 0) {
    pwm = 0;
  }
  analogWrite(PWM_PIN, pwm);
}

void taskController(void *pvParameters) { //Task que executa o calculo do controlador
  (void)pvParameters;

  for (;;) {
    // Se a tarefa está sendo acordada por um timer, a semáforo é dado para indicar que calculateControl pode ser chamado
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      startTimes[1] = xTaskGetTickCount();
      calculateControl();
      endTimes[1] = xTaskGetTickCount();
      sendData(1, startTimes[1], endTimes[1]);
    }

    vTaskDelay(1); // Pequeno atraso para liberar o processador
  }
}

void taskSetpoint(void *pvParameters) { // Task que coleta a entrada analógica do potenciometro e muda o setpoint da planta
  (void)pvParameters;

  for (;;) {
    // Se a tarefa está sendo acordada por um timer, o mutex é dado para indicar que o setpoint pode ser modificado
    if (xSemaphoreTake(xSetpointMutex, portMAX_DELAY) == pdTRUE) {
      startTimes[2] = xTaskGetTickCount();
      int analogValue = analogRead(A1);
      setpoint = map(analogValue, 0, 1020, 20, 80);
      endTimes[2] = xTaskGetTickCount();
      sendData(2, startTimes[2], endTimes[2]);
    }

    vTaskDelay(1); // Pequeno atraso para liberar o processador
  }
}

void sendData(int taskId, TickType_t startTime, TickType_t endTime) { // Funcao que serve para enviar os dados de temporizacao para motivos de plotangem
  const TickType_t ticksPerSecond = configTICK_RATE_HZ;

  Serial.print("TaskId:");
  Serial.print(taskId);
  Serial.print(", StartTime(s):");
  Serial.print(static_cast<float>(startTime) / ticksPerSecond, 8);  // Mostrar 3 casas decimais
  Serial.print(", EndTime(s):");
  Serial.println(static_cast<float>(endTime) / ticksPerSecond, 8);  // Mostrar 3 casas decimais
}

