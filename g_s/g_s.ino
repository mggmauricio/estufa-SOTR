#include <TaskScheduler.h>  // Substitua com a biblioteca RF real que você está usando

Scheduler scheduler;

float temperaturaAtual;  // Variável global para armazenar a temperatura medida
float pwmRecebido;      // Variável global para armazenar o PWM recebido do segundo microcontrolador

void readTemperatureTask() {
  // // Implemente a leitura do sensor de temperatura
  // temperaturaAtual = lerSensorTemperatura();
}

void sendTemperatureTask() {
  // // Envie os dados para o segundo microcontrolador por comunicação sem fio
  // enviarDadosParaMicrocontrolador2(temperaturaAtual);
}

void executeReceivedPWMTask() {
  // // Execute a lógica com base no PWM recebido do segundo microcontrolador
  // executarLogicaComPWM(pwmRecebido);
}

Task readTask(1000, TASK_FOREVER, &readTemperatureTask);
Task sendTask(1000, TASK_FOREVER, &sendTemperatureTask);
Task executePWM(1000, TASK_FOREVER, &executeReceivedPWMTask);

void setup() {
  // Inicialize o sensor de temperatura, módulo de comunicação sem fio, etc.
  // inicializarSensorTemperatura();
  // inicializarComunicacaoSemFio();

  scheduler.init();
  scheduler.addTask(readTask);
  scheduler.addTask(sendTask);
  scheduler.addTask(executePWM);
  
  // scheduler.setHighPriority(1, readTask);
  // scheduler.setLowPriority(2, sendTask);
  // scheduler.setLowPriority(3, executePWMTask);

  scheduler.startNow();
}

void loop() {
  scheduler.execute();
}
