/*
*Biblioteca criada por Alexandre Crepory Abbott de Oliveira para o Trabalho de Graduação do curso de Engenharia Mecatrônica da Universidade de Brasília
*Julho de 2017
*/

#include"Crepe.h"

//Pino do botão da placa central
#define botao A7
//Pinos para o controle do driver
const int m_B = 10;
const int m_esquerdo_frente = 6; 
const int m_esquerdo_tras = 3;
const int m_A = 11;
const int m_direita_frente = 12; 
const int m_direita_tras = 2;
const int stby = 13;
//Pino do buzzer da placa central
const int buzzer = 9;
//Endereço da UMI para a comunicação I2C
const int MPU=0x68;

//Construtor da classe Robo
Robo::Robo(void){
  //Motor da Esquerda
  pinMode (m_B, OUTPUT);
  pinMode (m_esquerdo_frente, OUTPUT);  //frente
  pinMode (m_esquerdo_tras, OUTPUT);  //tras

  //Motor da Direita
  pinMode (m_A, OUTPUT);
  pinMode (m_direita_frente, OUTPUT); //frente
  pinMode (m_direita_tras, OUTPUT); //tras

  //Stand-by do driver
  pinMode (stby, OUTPUT);
  digitalWrite (stby, HIGH);

  //Buzzer
  pinMode (buzzer, OUTPUT);

  //Botão
  pinMode (botao, INPUT);
}

bool Robo::LerBotao (void){ //Como o pino A7 não possui conversor analógico-digital é necessário fazer a transformação no software
  bool valor = 0;
  int inter;
  inter = analogRead (botao);
  if (inter > 900){
    valor = 1;
  }
  return (valor);
}

//Aciona o buzzer utilizando um PWM com valores entre 0 e 255
void Robo::TocarSom (int potencia){ 
  if ((potencia<256)&&(potencia>=0)){
    analogWrite (buzzer, potencia);
  }
}

//Aciona os pinos do driver para mover os motores para frente com a mesma potência, utilizando o mesmo valor de PWM para os dois motores
void Robo::IrFrente (int potencia) { 
  analogWrite (m_A, potencia);
  analogWrite (m_B, potencia);
  digitalWrite(m_esquerdo_frente, HIGH);
  digitalWrite(m_esquerdo_tras, LOW);
  digitalWrite(m_direita_frente, HIGH);
  digitalWrite(m_direita_tras, LOW);
}

//Aciona os pinos do driver para mover os motores para trás com a mesma potência, utilizando o mesmo valor de PWM para os dois motores
void Robo::IrTras (int potencia) { 
  analogWrite (m_A, potencia);
  analogWrite (m_B, potencia);
  digitalWrite(m_esquerdo_frente, LOW);
  digitalWrite(m_esquerdo_tras, HIGH);
  digitalWrite(m_direita_frente, LOW);
  digitalWrite(m_direita_tras, HIGH);
}

//Aciona os pinos do driver para girar os motores em direções opostas, girando o robô para a direita, com a mesma potência, utilizando o mesmo valor de PWM para os dois motores
void Robo::GirarDireita (int potencia) { 
  analogWrite (m_A, potencia);
  analogWrite (m_B, potencia);
  digitalWrite(m_esquerdo_frente, HIGH);
  digitalWrite(m_esquerdo_tras, LOW);
  digitalWrite(m_direita_frente, LOW);
  digitalWrite(m_direita_tras, HIGH);
}

//Aciona os pinos do driver para girar os motores em direções opostas, girando o robô para a esquerda, com a mesma potência, utilizando o mesmo valor de PWM para os dois motores
void Robo::GirarEsquerda (int potencia) { 
  analogWrite (m_A, potencia);
  analogWrite (m_B, potencia);
  digitalWrite(m_esquerdo_frente, LOW);
  digitalWrite(m_esquerdo_tras, HIGH);
  digitalWrite(m_direita_frente, HIGH);
  digitalWrite(m_direita_tras, LOW);
}

//Aciona os pinos do driver fazendo com que o robô pare
void Robo::Parar (void) {
  analogWrite (m_A, 0);
  analogWrite (m_B, 0);
  digitalWrite(m_esquerdo_frente, LOW);
  digitalWrite(m_esquerdo_tras, LOW);
  digitalWrite(m_direita_frente, LOW);
  digitalWrite(m_direita_tras, LOW);
}

//Construtor da classe Motor 
Motor::Motor (void){
  //MOTOR DA ESQUERDA
  pinMode (m_B, OUTPUT);
  pinMode (m_esquerdo_frente, OUTPUT);  //frente
  pinMode (m_esquerdo_tras, OUTPUT);  //tras

  //MOTOR DA DIREITA
  pinMode (m_A, OUTPUT);
  pinMode (m_direita_frente, OUTPUT); //frente
  pinMode (m_direita_tras, OUTPUT); //tras

  //STANDBY DO DRIVER
  pinMode (stby, OUTPUT);
  digitalWrite (stby, HIGH);
}

//Aciona os pinos do driver para mover apenas os motores selecionados (A, B ou AB) para frente
void Motor::IrFrente (int motor, int potencia){
  if (motor == 1) {
    analogWrite (m_A, potencia);
    digitalWrite(m_esquerdo_frente, HIGH);
    digitalWrite(m_esquerdo_tras, LOW);
  }
  if (motor == 2) {
    analogWrite (m_B, potencia);
    digitalWrite(m_direita_frente, HIGH);
    digitalWrite(m_direita_tras, LOW);
  }
  if (motor == 3) {
    analogWrite (m_A, potencia);
    analogWrite (m_B, potencia);
    digitalWrite(m_esquerdo_frente, HIGH);
    digitalWrite(m_esquerdo_tras, LOW);
    digitalWrite(m_direita_frente, HIGH);
    digitalWrite(m_direita_tras, LOW);
  }
}

//Aciona os pinos do driver para mover apenas os motores selecionados (A, B ou AB) para trás
void Motor::IrTras (int motor, int potencia){
  if (motor == 1) {
    analogWrite (m_A, potencia);
    digitalWrite(m_esquerdo_frente, LOW);
    digitalWrite(m_esquerdo_tras, HIGH);
  }
  if (motor == 2) {
    analogWrite (m_B, potencia);
    digitalWrite(m_direita_frente, LOW);
    digitalWrite(m_direita_tras, HIGH);
  }
   if (motor == 3) {
    analogWrite (m_A, potencia);
    analogWrite (m_B, potencia);
    digitalWrite(m_esquerdo_frente, LOW);
    digitalWrite(m_esquerdo_tras, HIGH);
    digitalWrite(m_direita_frente, LOW);
    digitalWrite(m_direita_tras, HIGH);
  }
}

//Aciona os pinos do driver para parar os motores selecionados (A, B ou AB)
void Motor::Parar (int motor){
  if (motor == 1) {
    analogWrite (m_A, 0);
    digitalWrite(m_esquerdo_frente, LOW);
    digitalWrite(m_esquerdo_tras, LOW);
  }
  if (motor == 2) {
    analogWrite (m_B, 0);
    digitalWrite(m_direita_frente, LOW);
    digitalWrite(m_direita_tras, LOW);
  }
  if (motor == 3) {
    analogWrite (m_A, 0);
    analogWrite (m_B, 0);
    digitalWrite(m_esquerdo_frente, LOW);
    digitalWrite(m_esquerdo_tras, LOW);
    digitalWrite(m_direita_frente, LOW);
    digitalWrite(m_direita_tras, LOW);
  }
}

//Construtor da classe do Sensor de Distância. Este sensor pode ser conectado nas portas 1, 2 , 3 e 4
SensorDistancia::SensorDistancia (int porta){
  port=porta; //a informação da porta que o sensor está é guardada na classe
  if (porta==1){
    Ultrasonic ultrasonic1(7, 8); //Usa a biblioteca pronta para o este sensor
  }
  if (porta==2){
    Ultrasonic ultrasonic2(4, 5);
  }
  if (porta==3){
    Ultrasonic ultrasonic3(A3, A2);
  }
  if (porta==4){
    Ultrasonic ultrasonic4(A5, A4);
  }
}

//Faz a leitura do sensor de distância, já tem a informação da sua porta guardada
float SensorDistancia::Ler(void){
  float distancia;
  long microsec;
  if (port == 1){
    Ultrasonic ultrasonic1(7, 8);
    microsec = ultrasonic1.timing(); //pega a informação de quanto demorou a onda
    distancia = ultrasonic1.convert(microsec, Ultrasonic::CM); //transforma este tempo em distância
  }
  if (port == 2){
    Ultrasonic ultrasonic2(4, 5);
    microsec = ultrasonic2.timing();
    distancia = ultrasonic2.convert(microsec, Ultrasonic::CM);
  }
  if (port == 3){
    Ultrasonic ultrasonic3(A3, A2);
    microsec = ultrasonic3.timing();
    distancia = ultrasonic3.convert(microsec, Ultrasonic::CM);
  }
  if (port == 4){
    Ultrasonic ultrasonic4(A5, A4);
    microsec = ultrasonic4.timing();
    distancia = ultrasonic4.convert(microsec, Ultrasonic::CM);
  }
  return (distancia);
}

//Construtor da classe do Sensor de Toque. Este sensor pode ser conectado nas portas 1, 2 , 3 e 4
SensorToque::SensorToque(int porta){
  port=porta;
  if (porta==1){
    pinMode (8, INPUT);
    pinMode (7, INPUT);
  }
  if (porta==2){
    pinMode (5, INPUT);
    pinMode (4, INPUT);
  }
  if (porta==3){
    pinMode (A2, INPUT);
    pinMode (A3, INPUT);
  }
  if (porta==4){
    pinMode (A4, INPUT);
    pinMode (A5, INPUT);
  }
}

//Faz a leitura do sensor de toque do lado A, já tem a informação da sua porta guardada
bool SensorToque::LerA(void){
  bool valor;
  if (port == 1){
    valor = digitalRead(8);
  }
  if (port == 2){
    valor = digitalRead (5);
  }
  if (port == 3){
    valor = digitalRead (A2);
  }
  if (port == 4){
    valor = digitalRead (A4);
  }
  return (valor);
}

//Faz a leitura do sensor de toque do lado B, já tem a informação da sua porta guardada
bool SensorToque::LerB(void){
  bool valor;
  if (port == 1){
    valor = digitalRead (7);
  }
  if (port == 2){
    valor = digitalRead (4);
  }
  if (port == 3){
    valor = digitalRead (A3);
  }
  if (port == 4){
    valor = digitalRead (A5);
  }
  return (valor);
}

//Construtor da classe do Sensor de Linha. Este sensor pode ser conectado nas portas 3 e 4 por precisar de pinos analógicos 
SensorLinha::SensorLinha(int porta){
  port = porta;
  if (porta==3){
    pinMode (A2, INPUT);
    pinMode (A3, INPUT);
  }
  if (porta==4){
    pinMode (A4, INPUT);
    pinMode (A5, INPUT);
  }
}

//Faz a leitura do sensor de linha do lado A, já tem a informação da sua porta guardada
int SensorLinha::LerA(void){
  int valor;
  if (port == 3){
    valor = analogRead (A3);
  }
  if (port == 4){
    valor = analogRead (A5);
  }
  return (valor);
}

//Faz a leitura do sensor de linha do lado B, já tem a informação da sua porta guardada
int SensorLinha::LerB(void){
  int valor;
  if (port == 3){
    valor = analogRead (A2);
  }
  if (port == 4){
    valor = analogRead (A4);
  }
  return (valor);
}

//Construtor da classe do Sensor de Direção. Este sensor pode ser conectado apenas na porta 4 por precisar dos pinos SDA e SCL 
SensorDirecao::SensorDirecao (int porta){ 
  port = porta;
  if (porta==4){
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0); 
    Wire.endTransmission(true);
  }
}

//Ler o valor do acelerômetro no eixo X 
int SensorDirecao::LerAcX(void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();  
    return (AcX);
  }
}

//Ler o valor do acelerômetro no eixo Y
int SensorDirecao::LerAcY(void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();
    return (AcY);
  }
}

//Ler o valor do acelerômetro no eixo Z
int SensorDirecao::LerAcZ (void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();
    return (AcZ);
  }
}

//Ler o valor do girômetro no eixo X
int SensorDirecao::LerGrX (void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();
    return (GyX);
  }
}

//Ler o valor do girômetro no eixo Y
int SensorDirecao::LerGrY (void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();
    return (GyY);
  }
}

//Ler o valor do girômetro no eixo Z
int SensorDirecao::LerGrZ (void){
  if (port = 4){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    //Solicita os dados do sensor
    Wire.requestFrom(MPU,14,true);  
    //Armazena o valor dos sensores nas variaveis correspondentes
    int AcX=Wire.read()<<8|Wire.read();       
    int AcY=Wire.read()<<8|Wire.read();  
    int AcZ=Wire.read()<<8|Wire.read();  
    int Tmp=Wire.read()<<8|Wire.read();  
    int GyX=Wire.read()<<8|Wire.read();  
    int GyY=Wire.read()<<8|Wire.read();  
    int GyZ=Wire.read()<<8|Wire.read();
    return (GyZ);
  }
}

