#include <TaskScheduler.h>  // Substitua com a biblioteca RF real que você está usando
#include <SoftwareSerial.h>


SoftwareSerial hc12(10,11);
Scheduler scheduler;

float temperaturaAtual;  // Variável global para armazenar a temperatura medida
float pwmRecebido;      // Variável global para armazenar o PWM recebido do segundo microcontrolador

void readTemperatureTask() {
  temperaturaAtual = analogRead(A0);
}

void sendTemperatureTask() {
  hc12.print(temperaturaAtual);
}

void executeReceivedPWMTask() {
  analogWrite(3, pwmRecebido);
}

void receivePWM(){
  pwmRecebido = hc12.parseFloat();
}

Task readTask(5, TASK_FOREVER, &readTemperatureTask);
Task sendTask(10, TASK_FOREVER, &sendTemperatureTask);
Task executePWM(20, TASK_FOREVER, &executeReceivedPWMTask);

void setup() {

  hc12.begin(9600);
  scheduler.init();
  scheduler.addTask(readTask);
  scheduler.addTask(sendTask);
  scheduler.addTask(executePWM);


  scheduler.startNow();
}

void loop() {
  scheduler.execute();
}
