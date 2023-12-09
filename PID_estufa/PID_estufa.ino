// Controlando a temperatura interna de uma estufa utilizando uma resistencia de 5W
// e um sensor de temperatura LM35
// Gabriel Niederauer e Paulo Augusto Fronza

#define sensor A0 //Entrada sensor de temperatura: A0
#define controle 11 //Saída PWM: Pino 11
#define botaoMais 13 //Define pino 13 como botão mais
#define botaoMenos 12 //Define pino 12 como botão menos

unsigned long tempoInicioBmais = 0;
unsigned long tempoInicioBmenos = 0;
unsigned long tempoBotaoMais = 0;
unsigned long tempoBotaoMenos = 0;
bool estadoBotaoMais;
bool estadoBotaoMenos;
bool estadoBotaoMaisAnt;
bool estadoBotaoMenosAnt;
int minutos;
int segundos;
int segundos_limit;

float temperatura;
float temperatura_anterior;
long ultimo_processo;
int diferenca_tempo;
float set_point = 65;
float erro = 0;
float PID;
int pwm; //0 a 255
int pwm_porcent;
float P, I, D;

float kp = 0.10848, ki = 0.01165, kd = 0;



void setup() {
  Serial.begin(9600);

  pinMode(botaoMais, INPUT_PULLUP);
  pinMode(botaoMenos, INPUT_PULLUP);

  pinMode(sensor, INPUT);
  pinMode(controle, OUTPUT);

}

void loop() {
  //Lê temperatura a ccada 1 segundo e printa na serial
  delay(1000);
  temperatura = (float(analogRead(sensor))*5/(1023))/0.01;
  //Serial.print("  Temperatura: ");
  Serial.println(temperatura);
  

  //Definição do setpoint
  estadoBotaoMais = !digitalRead(botaoMais);
  estadoBotaoMenos = !digitalRead(botaoMenos);

  if (estadoBotaoMais && !estadoBotaoMaisAnt) {
    set_point = set_point + 5;
  } 

  if (estadoBotaoMenos && !estadoBotaoMenosAnt) {
    set_point = set_point - 5;
  } 

  if(set_point >= 80){
    set_point = 80;
  }
  if(set_point <= 15){
    set_point = 15;
  }


  //Implementando PID
  diferenca_tempo = (millis() - ultimo_processo) / 1000.0;
  ultimo_processo = millis();

  erro = set_point - temperatura;

  P = erro * kp;

  I = (I + erro * ki) * diferenca_tempo;

  D = (temperatura_anterior - temperatura) * kd * diferenca_tempo;

  temperatura_anterior = temperatura;

  PID = P + I + D;

  //Serial.print("  PID: ");
  //Serial.print(PID);

  //Gerando PWM a partir do PID
  //pwm = PID;
  pwm = map(PID, -2, 90, 0, 255);

  if(pwm >= 255){
    pwm = 255;
  }
  if(pwm <= 0){
    pwm = 0;
  }

  pwm_porcent = map(pwm, 0, 255, 0, 100);

  segundos_limit += 1;
  if(segundos_limit > 59){
    segundos_limit = 0;
  }
  
  segundos = segundos +1;
  minutos = segundos / 60;

  //Saída de controle
  analogWrite(controle, pwm);
}
