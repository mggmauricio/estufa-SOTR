#include <TaskScheduler.h>

void receiveDataTaskCallback() {
  // Implemente a lógica de recebimento de dados do Microcontrolador 1
}

void controlTaskCallback() {
  // Implemente a lógica do controle, receba o dado do Microcontrolador 1, execute o controlador e envie o PWM para o Microcontrolador 3
}

void pwmSendTaskCallback() {
  // Implemente a lógica de envio do PWM para o Microcontrolador 3
}

Scheduler scheduler;

Task receiveDataTask(100, TASK_FOREVER, &receiveDataTaskCallback);  // Tarefa de leitura do sensor
Task controlTask(200, TASK_FOREVER, &controlTaskCallback);          // Tarefa de controle
Task pwmSendTask(300, TASK_FOREVER, &pwmSendTaskCallback);          // Tarefa de envio do PWM

void setup() {
  scheduler.init();
  scheduler.addTask(receiveDataTask);
  scheduler.addTask(controlTask);
  scheduler.addTask(pwmSendTask);
  scheduler.startNow();
}

void loop() {
  scheduler.execute();
}
