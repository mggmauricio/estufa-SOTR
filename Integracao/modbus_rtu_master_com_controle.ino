
#include<SoftwareSerial.h>
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define botaoMais 13 //Define pino 13 como botão mais
#define botaoMenos 12 //Define pino 12 como botão menos



unsigned long interframe_delay = 2; 
int preset_multiple_registers(int slave, int start_addr,int reg_count, int *data);
int read_holding_registers(int slave, int start_addr, int count,int *dest, int dest_size);

SoftwareSerial hc12(10,11);

LiquidCrystal_I2C lcd(0x38,16,2); //define display

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
float set_point = 65.0;
float erro = 0;
float PID;
int pwm; //0 a 255
int pwm_porcent;
float P, I, D;

float x1 = 0.0;
float b0 = 0.10243; 
float b1 = 0.005848753; 
float b2 = -0.096581247;
float a1 = 0.94186, a2 = -0.0574;
float setpoint = 30.0;
float y1 = 0.0;
static float ek1 = 0, ek2 = 0;
static float uk1 = 0, uk2 = 0;


int controller(float setpoint, float x1){
  float ek0 = setpoint - x1;
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
  Serial.print("y1 controller: ");
  Serial.println(y1);
  Serial.print("PWM:");
  Serial.println(pwm);
  return pwm;
}


////Fim da parte do codigo de controle original:

void setup()
{
        const int baudrate = 9600;
        if (baudrate <= 19200)
                interframe_delay = (unsigned long)(3.5 * 11 / baudrate);  /* Modbus t3.5 */
        Serial.begin(baudrate);
        hc12.begin(baudrate); 	/* format 8N1, DOES NOT comply with Modbus spec. */

        //Parte do codigo de controle original:
        pinMode(botaoMais, INPUT_PULLUP);
        pinMode(botaoMenos, INPUT_PULLUP);
        lcd.init(); // Inicializando o LCD
        lcd.backlight(); // Ligando o BackLight do LCD
        lcd.clear(); // LIMPA O DISPLAY
        



        //Fim da parte do codigo de controle original:
}

/* example data */
int retval = 0;
int data[10];
int retry[10];




void loop()
{
        /* example, this will write some data in the first 10 registers of slave 1  */
                read_holding_registers(1,1,1,retry,10);
                delay(30); 
temperatura = (float(retry[0])*5/(1023))/0.01;
                Serial.println(retry[0]);
                //Definição do setpoint
  estadoBotaoMais = !digitalRead(botaoMais);
  estadoBotaoMenos = !digitalRead(botaoMenos);

  if(estadoBotaoMais && !estadoBotaoMaisAnt){
    set_point = set_point + 5.0;
  } 

  if (estadoBotaoMenos && !estadoBotaoMenosAnt) {
    set_point = set_point - 5.0;
  } 

  if(set_point >= 80.0){
    set_point = 80.0;
  }
  if(set_point <= 15){
    set_point = 15.0;
  }
// 
  pwm = controller(set_point, temperatura);

  pwm_porcent = map(pwm, 0, 255, 0, 100);
/*
  Serial.print("  PWM: ");
  Serial.print(pwm_porcent);
  Serial.print(" % ");
*/
  // segundos_limit += 1;
  // if(segundos_limit > 59){
  //   segundos_limit = 0;
  // }
  
  // segundos = segundos +1;
  // minutos = segundos / 60;
/*
  Serial.print("  Tempo: ");
  Serial.print(minutos);
  Serial.print(":");
  Serial.println(segundos_limit);
*/
  //Saída de controle
  data[0] = pwm;
  preset_multiple_registers(2,1,1, data);
  

  lcd.clear();                          //formatando display
  lcd.print("Temperatura");                 
  lcd.setCursor(13,0);
  lcd.print(round(temperatura));
  lcd.setCursor(15,0);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("SetPoint");
  lcd.setCursor(13,1);
  lcd.print(set_point);
  lcd.setCursor(15,1);
  lcd.print("C");
  
                
}




/****************************************************************************
 * BEGIN MODBUS RTU MASTER FUNCTIONS
 ****************************************************************************/

#define TIMEOUT 1000		/* 1 second */
#define MAX_READ_REGS 125
#define MAX_WRITE_REGS 125
#define MAX_RESPONSE_LENGTH 256
#define PRESET_QUERY_SIZE 256
/* errors */
#define PORT_ERROR -5

/*
CRC
 
 INPUTS:
 	buf   ->  Array containing message to be sent to controller.            
 	start ->  Start of loop in crc counter, usually 0.
 	cnt   ->  Amount of bytes in message being sent to controller/
 OUTPUTS:
 	temp  ->  Returns crc byte for message.
 COMMENTS:
 	This routine calculates the crc high and low byte of a message.
 	Note that this crc is only used for Modbus, not Modbus+ etc. 
 ****************************************************************************/

unsigned int crc(unsigned char *buf, int start, int cnt)
{
        int i, j;
        unsigned temp, temp2, flag;

        temp = 0xFFFF;

        for (i = start; i < cnt; i++) {
                temp = temp ^ buf[i];

                for (j = 1; j <= 8; j++) {
                        flag = temp & 0x0001;
                        temp = temp >> 1;
                        if (flag)
                                temp = temp ^ 0xA001;
                }
        }

        /* Reverse byte order. */

        temp2 = temp >> 8;
        temp = (temp << 8) | temp2;
        temp &= 0xFFFF;

        return (temp);
}


/***********************************************************************
 * 
 * 	The following functions construct the required query into
 * 	a modbus query packet.
 * 
 ***********************************************************************/

#define REQUEST_QUERY_SIZE 6	/* the following packets require          */
#define CHECKSUM_SIZE 2		/* 6 unsigned chars for the packet plus   */
/* 2 for the checksum.                    */

void build_request_packet(int slave, int function, int start_addr,
int count, unsigned char *packet)
{
        packet[0] = slave;
        packet[1] = function;
        start_addr -= 1;
        packet[2] = start_addr >> 8;
        packet[3] = start_addr & 0x00ff;
        packet[4] = count >> 8;
        packet[5] = count & 0x00ff;
}

/*************************************************************************
 * 
 * modbus_query( packet, length)
 * 
 * Function to add a checksum to the end of a packet.
 * Please note that the packet array must be at least 2 fields longer than
 * string_length.
 **************************************************************************/

void modbus_query(unsigned char *packet, size_t string_length)
{
        int temp_crc;

        temp_crc = crc(packet, 0, string_length);

        packet[string_length++] = temp_crc >> 8;
        packet[string_length++] = temp_crc & 0x00FF;
        packet[string_length] = 0;
}



/***********************************************************************
 * 
 * send_query(query_string, query_length )
 * 
 * Function to send a query out to a modbus slave.
 ************************************************************************/

int send_query(unsigned char *query, size_t string_length)
{

        int i;

        modbus_query(query, string_length);
        string_length += 2;

        for (i = 0; i < string_length; i++) {
                hc12.write(query[i]);
        }
        /* without the following delay, the reading of the response might be wrong 
        * apparently, */
        delay(200);            /* FIXME: value to use? */

        return i; 		/* it does not mean that the write was succesful, though */
}


/***********************************************************************
 * 
 * 	receive_response( array_for_data )
 * 
 * Function to monitor for the reply from the modbus slave.
 * This function blocks for timeout seconds if there is no reply.
 * 
 * Returns:	Total number of characters received.
 ***********************************************************************/

int receive_response(unsigned char *received_string)
{

        int bytes_received = 0;
        int i = 0;
        /* wait for a response; this will block! */

        while(hc12.available() == 0) {
                delay(1);
                if (i++ > TIMEOUT)
                        return bytes_received;
        }

        /* FIXME: does Serial.available wait 1.5T or 3.5T before exiting the loop? */
        while(hc12.available()) {
                received_string[bytes_received] = hc12.read();
                bytes_received++;
                if (bytes_received >= MAX_RESPONSE_LENGTH)
                        return PORT_ERROR;
        }    

        return (bytes_received);
}


/*********************************************************************
 * 
 * 	modbus_response( response_data_array, query_array )
 * 
 * Function to the correct response is returned and that the checksum
 * is correct.
 * 
 * Returns:	string_length if OK
 * 		0 if failed
 * 		Less than 0 for exception errors
 * 
 * 	Note: All functions used for sending or receiving data via
 * 	      modbus return these return values.
 * 
 **********************************************************************/

int modbus_response(unsigned char *data, unsigned char *query)
{
        int response_length;

        unsigned int crc_calc = 0;
        unsigned int crc_received = 0;
        unsigned char recv_crc_hi;
        unsigned char recv_crc_lo;

      do {        // repeat if unexpected slave replied
                response_length = receive_response(data);
      } while ((response_length > 0) && (data[0] != query[0]));

        if (response_length) {


                crc_calc = crc(data, 0, response_length - 2);

                recv_crc_hi = (unsigned) data[response_length - 2];
                recv_crc_lo = (unsigned) data[response_length - 1];

                crc_received = data[response_length - 2];
                crc_received = (unsigned) crc_received << 8;
                crc_received =
                        crc_received | (unsigned) data[response_length - 1];


                /*********** check CRC of response ************/

                if (crc_calc != crc_received) {
                        response_length = 0;
                }



                /********** check for exception response *****/

                if (response_length && data[1] != query[1]) {
                        response_length = 0 - data[2];
                }
        }
        return (response_length);
}


/************************************************************************
 * 
 * 	read_reg_response
 * 
 * 	reads the response data from a slave and puts the data into an
 * 	array.
 * 
 ************************************************************************/

int read_reg_response(int *dest, int dest_size, unsigned char *query)
{

        unsigned char data[MAX_RESPONSE_LENGTH];
        int raw_response_length;
        int temp, i;

        raw_response_length = modbus_response(data, query);
        if (raw_response_length > 0)
                raw_response_length -= 2;

        if (raw_response_length > 0) {
                /* FIXME: data[2] * 2 ???!!! data[2] isn't already the byte count (number of registers * 2)?! */
                for (i = 0;
		     i < (data[2] * 2) && i < (raw_response_length / 2);
		     i++) {

                        /* shift reg hi_byte to temp */
                        temp = data[3 + i * 2] << 8;
                        /* OR with lo_byte           */
                        temp = temp | data[4 + i * 2];

                        dest[i] = temp;
                }
        }
        return (raw_response_length);
}


/***********************************************************************
 * 
 * 	preset_response
 * 
 * 	Gets the raw data from the input stream.
 * 
 ***********************************************************************/

int preset_response(unsigned char *query)
{
        unsigned char data[MAX_RESPONSE_LENGTH];
        int raw_response_length;

        raw_response_length = modbus_response(data, query);

        return (raw_response_length);
}


/************************************************************************
 * 
 * 	read_holding_registers
 * 
 * 	Read the holding registers in a slave and put the data into
 * 	an array.
 * 
 *************************************************************************/

int read_holding_registers(int slave, int start_addr, int count,
int *dest, int dest_size)
{
        int function = 0x03; 	/* Function: Read Holding Registers */
        int ret;

        unsigned char packet[REQUEST_QUERY_SIZE + CHECKSUM_SIZE];

        if (count > MAX_READ_REGS) {
                count = MAX_READ_REGS;
        }

        build_request_packet(slave, function, start_addr, count, packet);

        if (send_query(packet, REQUEST_QUERY_SIZE) > -1) {
                ret = read_reg_response(dest, dest_size, packet);
        } 
        else {

                ret = -1;
        }

        return (ret);
}


/************************************************************************
 * 
 * 	preset_multiple_registers
 * 
 * 	Write the data from an array into the holding registers of a
 * 	slave.
 * 
 *************************************************************************/

int preset_multiple_registers(int slave, int start_addr,
int reg_count, int *data)
{
        int function = 0x10; 	/* Function 16: Write Multiple Registers */
        int byte_count, i, packet_size = 6;
        int ret;

        unsigned char packet[PRESET_QUERY_SIZE];

        if (reg_count > MAX_WRITE_REGS) {
                reg_count = MAX_WRITE_REGS;
        }

        build_request_packet(slave, function, start_addr, reg_count, packet);
        byte_count = reg_count * 2;
        packet[6] = (unsigned char)byte_count;

        for (i = 0; i < reg_count; i++) {
                packet_size++;
                packet[packet_size] = data[i] >> 8;
                packet_size++;
                packet[packet_size] = data[i] & 0x00FF;
        }

        packet_size++;
        if (send_query(packet, packet_size) > -1) {
                ret = preset_response(packet);
        } 
        else {
                ret = -1;
        }

        return (ret);
}



