#include <TaskScheduler.h>
// #include <VirtualWire.h>

// Crie um objeto Scheduler


void pwmReceiveTaskCallback() {
  // Implemente a lógica de recepção do PWM enviado pelo Microcontrolador 2
}

// Defina as tarefas
Task pwmReceiveTask(100, TASK_FOREVER, &pwmReceiveTaskCallback);
Scheduler scheduler;

void setup() {
  // Inicialize o objeto Scheduler
  scheduler.init();
  // Adicione as tarefas ao Scheduler
  scheduler.addTask(pwmReceiveTask);
  // Inicie o Scheduler
  scheduler.startNow();
}

void loop() {
  // Execute o Scheduler
  scheduler.execute();
}

