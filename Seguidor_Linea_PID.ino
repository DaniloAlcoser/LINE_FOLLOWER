//SENSORES//

#include <QTRSensors.h>  //Librería

QTRSensors qtr;

const uint8_t SensorCount = 8;    //Número de Sensores
uint16_t sensorValues[SensorCount];  

//DECARACION DE VARIABLES//

#define  pin_pwma  10
#define  ain01  9
#define  ain02  8
#define  pin_pwmb  3
#define  bin01  5
#define  bin02  4
#define  led  7
#define  boton  2
#define  dip1  11
#define  dip2  12
int sp = 3500;      //Set Point
int estado_btn;     
int contador = 1;     //Estado Inicial para el pulsador
int estado_dip1;      //Estado DipSwitch 1
int estado_dip2;      //Estado DipSwitch 2

//Contantes PID ; 

// float kp = 0.01;    //kp = error porporcional
// float kd = 1;       //kd = derivativo
// float ki = 0.002;   //ki = integral

float kp = 0.055;    //kp = error porporcional
float kd = 1.5;       //kd = derivativo
float ki = 0.002;   //ki = integral

int vel;    //Velocidad

//Velocidad Frenos
int veladelante1=220; 
int veldetras1=110;
int veladelante2=255; 
int veldetras2=90;  

uint16_t position;    //Posición del Sensor

//Datos para la integral
int error1=0;
int error2=0;
int error3=0;
int error4=0;
int error5=0;
int error6=0;

//Variables PID
int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop;    //Proporcional Anterior

/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

//Frecuencia de PWM para pines D3 & D11
TCCR2B = TCCR2B & B11111000 | B00000001;
//set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

// Frecuencia de PWM para pines D9 & D10
TCCR1B = TCCR1B & B11111000 | B00000001;    
// Set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz

  Serial.begin(9600);

  pinMode(boton,INPUT);   //Inicio
  digitalWrite(boton,LOW);
  pinMode(ain01, OUTPUT);
  pinMode(ain02, OUTPUT);
  pinMode(bin01, OUTPUT);
  pinMode(bin02, OUTPUT); 
  pinMode(dip1,INPUT);   
  pinMode(dip2, INPUT);
  pinMode(6, OUTPUT);   //Stby
  digitalWrite(6,HIGH);

//MODOS//

estado_dip1 = digitalRead(dip1);
estado_dip2 = digitalRead(dip2);

    if ((estado_dip1 == LOW) && (estado_dip2 == LOW)){
   vel=150;   //Velocidad Baja
    }

    if ((estado_dip1 == HIGH) && (estado_dip2 == LOW)){
   vel=150;   //SOLO PID 
    }

   if ((estado_dip1 == LOW) && (estado_dip2 == HIGH)){
   vel=150;   //SOLO PID 
    }

   if ((estado_dip1 == HIGH) && (estado_dip2 == HIGH)){
   vel=255;   //Máxima Velocidad
    }


//CONFIGURACION Y CALIBRACIÓN DE SENSORES//

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7,A6, A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(13);

  delay(500);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  for (uint16_t i = 0; i < 150; i++)
  {
    qtr.calibrate();
  }
    digitalWrite(led, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

//POSICIÓN DE LOS SENSORES (QTR-8A)//

  position = qtr.readLineBlack(sensorValues);
  
//INICIO//

  estado_btn = digitalRead(boton);
  
  while(estado_btn == HIGH){
    contador = 2;
    estado_btn = digitalRead(boton);      
    }

//Velocidad de Motores
if(contador == 1)  {
    motores(0,0);
    }

if(contador == 2)  {
    PID();
    frenos();   
    }

//       Serial.println(position);
//       Serial.print(",");
            
}

//FUNCIÓN PID//

void PID() {

proporcional = position - sp;   //Proporcional = Posición - Set Point
derivativo = proporcional - last_prop;    //Derivatico = Error - Error Anterior
integral = error1 + error2 + error3 + error4 + error5 + error6;   //Integral = Acumulación de Errores

last_prop = proporcional;   //Actualización de Proporcional

error6 = error5;
error5 = error4;
error4 = error3;
error3 = error2;
error2 = error1;
error1 = proporcional;

int PID = (proporcional*kp) + (derivativo*kd) + (integral*ki);    //Control PID

if(PID > vel) PID = vel;
else if(PID < -vel) PID = -vel;

//Control de Velovidades en los Motores
(PID < 0)?    //Condicional
motores(vel, vel + PID) : motores(vel - PID, vel);   //valor_si_verdadero : valor_si_falso
//motores(valor derecho, valor izquierdo)
}


//FUNCIÓN FRENOS//

void frenos(){

if(position>=1000 && position<=1500){
  motores(veladelante1, veldetras1);
}
if(position>=5500 && position<=6000){
  motores(veldetras1, veladelante1);
}
if(position>=0 && position<1000){
  motores(veladelante2, -veldetras2);
}
if(position>6000 && position<=7000){
  motores(-veldetras2, veladelante2);

}

  }

//FUNCIÓN MOTORES (Adelante = 0 hasta 255)  (Atras = -1 hasta -255)//

void motores(int right, int left){

//MOTOR DERECHO

// Adelante
if(right>=0){

digitalWrite(ain01,HIGH);
digitalWrite(ain02,LOW);
}
//Atras
else{
digitalWrite(ain01,LOW);
digitalWrite(ain02,HIGH);
right = right*(-1);   //Convertimos a positivo nuestro valor de pwm
}
analogWrite(pin_pwma,right);   //Velocidad Motor Derecho

//MOTOR IZQUIERDO

//Adelante
if(left >= 0){
digitalWrite(bin01,HIGH);
digitalWrite(bin02,LOW);
}
//Atras
else{
digitalWrite(bin01,LOW);
digitalWrite(bin02,HIGH);
left = left*(-1);   //Convertimos a positivo nuestro valor de pwm
}
analogWrite(pin_pwmb,left);   //Velocidad Motor Izquierdo

}
