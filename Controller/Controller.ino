#include <TaskScheduler.h>

Scheduler scheduler;

float temperaturaRecebida;  // Variável global para armazenar a temperatura recebida
float pwmCalculado;        // Variável global para armazenar o PWM calculado

void receiveDataTask() {
  // Receba os dados do primeiro microcontrolador por comunicação sem fio
  // temperaturaRecebida = receberDadosDoMicrocontrolador1();
}

void calculatePWMTask() {
  // Realize o cálculo do PWM com base na temperatura recebida
  // pwmCalculado = calcularPWM(temperaturaRecebida);
}

void sendPWMTask() {
  // Envie o PWM calculado para o primeiro microcontrolador por comunicação sem fio
  // enviarPWMParaMicrocontrolador1(pwmCalculado);
}

void updateDisplayTask() {
  // Atualize o display com os dados recebidos e o PWM calculado
  // atualizarDisplay(temperaturaRecebida, pwmCalculado);
}

Task receiveData(1000, TASK_FOREVER, &receiveDataTask);
Task calculatePWM(1000, TASK_FOREVER, &calculatePWMTask);
Task sendPWM(1000, TASK_FOREVER, &sendPWMTask);
Task display(1000, TASK_FOREVER, &updateDisplayTask);

void setup() {
  // Inicialize o módulo de comunicação sem fio, display, etc.
  // inicializarComunicacaoSemFio();
  // inicializarDisplay();

  scheduler.init();
  scheduler.addTask(receiveData);
  scheduler.addTask(calculatePWM);
  scheduler.addTask(sendPWM);
  scheduler.addTask(display);

  // scheduler.setHighPriority(1, receiveDataTask);
  // scheduler.setMediumPriority(2, calculatePWMTask);
  // scheduler.setMediumPriority(3, sendPWMTask);
  // scheduler.setLowPriority(4, displayTask);

  scheduler.startNow();
}

void loop() {
  scheduler.execute();
}
