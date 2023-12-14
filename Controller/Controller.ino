#include <TaskScheduler.h>
#include<SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>


Scheduler scheduler;

  // Variável global para armazenar a temperatura recebida
float pwmCalculado;        // Variável global para armazenar o PWM calculado
float x1 = 0.0;
float b0 = 0.10243; 
float b1 = 0.005848753; 
float b2 = -0.096581247;
float a1 = 0.94186, a2 = -0.0574;
float setpoint = 30.0;
static float ek1 = 0, ek2 = 0;
static float uk1 = 0, uk2 = 0;

SoftwareSerial hc12(10,11);

LiquidCrystal_I2C lcd(0x38,16,2);


void receiveDataTask() {
  if (hc12.available() > 0){
    x1 = hc12.parseFloat();
  } 
}


void calculatePWMTask() {
  float ek0 = setpoint - x1;
  float uk0 = a1*uk1 + a2*uk2 + b0*ek0 + b1*ek1 + b2*ek2;
  uk2 = uk1;
  uk1 = uk0;
  ek2 = ek1;
  ek1 = ek0;
  float y1 = uk0;
  pwmCalculado = map(y1*100, -7, 18, 0, 255);
  if (pwmCalculado >= 255){
    pwmCalculado = 255;
  }
  else if(pwmCalculado <= 0){
    pwmCalculado = 0;

  }
}

void sendPWMTask() {
  hc12.print(pwmCalculado);

  Serial.print("PWM Enviado");
  Serial.println(pwmCalculado);
  // Envie o PWM calculado para o primeiro microcontrolador por comunicação sem fio
  // enviarPWMParaMicrocontrolador1(pwmCalculado);
}

void updateDisplayTask() {
  lcd.clear();             
  lcd.print("Temperatura");                 
  lcd.setCursor(13,0);
  lcd.print(round(x1));
  lcd.setCursor(15,0);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("SetPoint");
  lcd.setCursor(13,1);
  lcd.print(setpoint);
  lcd.setCursor(15,1);
  lcd.print("C");
}

Task receiveData(10, TASK_FOREVER, &receiveDataTask);
Task calculatePWM(20, TASK_FOREVER, &calculatePWMTask);
Task sendPWM(21, TASK_FOREVER, &sendPWMTask);
Task display(50, TASK_FOREVER, &updateDisplayTask);

void setup() {

  scheduler.init();
  scheduler.addTask(receiveData);
  scheduler.addTask(calculatePWM);
  scheduler.addTask(sendPWM);
  scheduler.addTask(display);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  scheduler.startNow();
}

void loop() {
  scheduler.execute();
}
