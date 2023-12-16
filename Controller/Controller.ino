#include <Arduino_FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>
// Inclua aqui as bibliotecas específicas para o seu hardware, como para o sensor, controlador PID, PWM e display.

// Defina os pinos para o sensor, PWM e display.
#define SENSOR_PIN A0
#define PWM_PIN 3
#define UP_PIN 13
#define LOW_PIN 12
#define SETPOINT A1


// Defina as variáveis globais necessárias para o seu aplicativo.
float sensorValue = 0.0;
// float setpoint = 25.0; // Exemplo de valor de referência para o controlador PID.

// Defina o tamanho da fila e a fila em si.
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
LiquidCrystal_I2C lcd(0x38,16,2);
// Protótipos de funções para as tarefas.
void taskReadSensor(void *pvParameters);
void taskController(void *pvParameters);
void taskPWM(void *pvParameters);
void taskSetpoint(void *pvParameters);
void taskUpdateDisplay(void *pvParameters);
void up_set();
void down_set();

void setup() {
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(LOW_PIN, INPUT_PULLUP);
  pinMode(A1, INPUT);    
  Serial.begin(9600);
  // Inicialize aqui os componentes específicos, como o sensor, PWM e display.
  // Crie a fila.
  sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
  digitalWrite(2, HIGH);
  // Crie as tarefas.
  xTaskCreate(taskReadSensor, "ReadSensor", 128, NULL, 2, NULL);
  xTaskCreate(taskController, "Controller", 128, NULL, 2, NULL);
  xTaskCreate(taskPWM, "PWM", 128, NULL, 2, NULL);
  xTaskCreate(taskSetpoint, "Setpoint", 128, NULL, 2, NULL);
  // lcd.init(); // Inicializando o LCD
  // lcd.backlight(); // Ligando o BackLight do LCD
  // lcd.clear();
  // xTaskCreate(taskUpdateDisplay, "UpdateDisplay", 128, NULL, 2, NULL);

  // Inicie o scheduler.
  vTaskStartScheduler();
}

void loop() {
  // analogWrite(PWM_PIN, 180);
  // O loop não será executado, pois o FreeRTOS cuidará das tarefas.
}




void taskSetpoint(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Leia o valor do sensor.
    // Serial.print("Leitura");
    // Serial.println(aloha);
    setpoint = map (analogRead(A1), 0, 1020, 20, 80);
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
    // Serial.print("Temperatura: ");
    // Serial.println(sensorValue);
    // Envie o valor do sensor para a fila.
    xQueueSend(sensorQueue, &setpoint, portMAX_DELAY);

    // Aguarde um curto período antes de ler novamente.
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}


void taskReadSensor(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    // Leia o valor do sensor.  
    sensorValue = (float(analogRead(SENSOR_PIN))*5/(1023))/0.01;
    Serial.print("Sensor: ");
    Serial.println(sensorValue);
    // Envie o valor do sensor para a fila.
    xQueueSend(sensorQueue, &sensorValue, portMAX_DELAY);

    // Aguarde um curto período antes de ler novamente.
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskController(void *pvParameters) {
  (void) pvParameters;

  float receivedValue;

  for (;;) {
    // Aguarde até que um valor seja recebido na fila.
    if (xQueueReceive(sensorQueue, &receivedValue, portMAX_DELAY)) {
        float ek0 = (float)setpoint - sensorValue;
        // Serial.println(setpoint);
        float uk0 = a1*uk1 + a2*uk2 + b0*ek0 + b1*ek1 + b2*ek2;
        uk2 = uk1;
        uk1 = uk0;
        ek2 = ek1;
        ek1 = ek0;
        y1 = uk0;

        pwm = map(y1*100, -7, 18, 0, 255);
        if (pwm >= 255){
          pwm = 255;
        }
        else if(pwm <= 0){
          pwm = 0;
        }
        Serial.println(y1);
        Serial.println(pwm);
        // Serial.println(pwm);
    analogWrite(PWM_PIN, pwm);
        
      // Implemente o algoritmo de controle PID aqui usando receivedValue e setpoint.
    }

    // Aguarde um curto período antes de calcular novamente.
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void taskPWM(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    
  }
}

// void taskUpdateDisplay(void *pvParameters) {
//   (void) pvParameters;

//   for (;;) {
//     lcd.clear();                          //formatando display
//     lcd.print("Temperatura");                 
//     lcd.setCursor(13,0);
//     lcd.print(round(sensorValue));
//     lcd.setCursor(15,0);
//     lcd.print("C");
//     lcd.setCursor(0,1);
//     lcd.print("SetPoint");
//     lcd.setCursor(13,1);
//     lcd.print(setpoint);
//     lcd.setCursor(15,1);
//     lcd.print("C");
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }
