#include <TaskScheduler.h>

// Crie um objeto Scheduler
Scheduler scheduler;


void sensorReadTaskCallback() {
  // Implemente a lógica de leitura do sensor e envio do dado para o Microcontrolador 2
}

void sendDataTaskCallback() {
  // Implemente a lógica de envio de dados para o Microcontrolador 2
}


// Defina as tarefas
Task sensorReadTask(100, TASK_FOREVER, &sensorReadTaskCallback);
Task sendDataTask(200, TASK_FOREVER, &sendDataTaskCallback);

void setup() {
  // Inicialize o objeto Scheduler
  scheduler.init();
  // Adicione as tarefas ao Scheduler
  scheduler.addTask(sensorReadTask);
  scheduler.addTask(sendDataTask);
  // Inicie o Scheduler
  scheduler.startNow();
}

void loop() {
  // Execute o Scheduler
  scheduler.execute();
}

