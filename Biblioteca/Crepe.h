/*
*Biblioteca criada por Alexandre Crepory Abbott de Oliveira para o Trabalho de Graduação do curso de Engenharia Mecatrônica da Universidade de Brasília
*Julho de 2017
*/
#ifndef CREPE_H
#define CREPE_H

#include <Arduino.h>
#include <Wire.h>
#include <Ultrasonic.h>

#define A 1
#define B 2
#define AB 3

class Robo{ //Classe responsável pelo controle dos motores, do botão e do buzzer
public:
  Robo(void); //construtor da classe
  void IrFrente (int potencia); //função para mover o robô para frente
  void IrTras (int potencia); //função para mover o robô para trás
  void GirarDireita (int potencia); //função para girar o robô para direita no seu próprio eixo
  void GirarEsquerda (int potencia); //função para girar o robô para esquerda no seu próprio eixo
  void Parar (void); //função para parar o robô
  bool LerBotao (void); //função para ler o estado do botão da placa central
  void TocarSom (int potencia); //função para emitir som usando o buzzer da placa central
private:
};

class Motor{ //Classe responsavel pela controle dos motores
public:
  Motor (void); //construtor da classe
  void IrFrente (int motor, int potencia); //função para girar o motor para frente
  void IrTras (int motor, int potencia); //função para girar o motor para trás
  void Parar (int motor); //função para parar o motor
private:
};

class SensorDistancia{ //Classe do Sensor de Distância
public:
  SensorDistancia (int porta); //construtor da classe
  float Ler (void); //função de leitura da distância em cm
private:
  int port; //variável para guardar a porta em que o sensor está conectado
};

class SensorToque{ //Classe do Sensor de Toque
public:
  SensorToque (int porta); //construtor da classe
  bool LerA (void); //função de leitura do sensor do lado A
  bool LerB (void); //função de leitura do sensor do lado B
private:
  int port; //variável para guardar a porta em que o sensor está conectado
};

class SensorLinha{ //Classe do Sensor de Linha
public:
  SensorLinha (int porta); //construtor da classe
  int LerA (void); //função de leitura do sensor do lado A
  int LerB (void); //função de leitura do sensor do lado B
private:
  int port; //variável para guardar a porta em que o sensor está conectado
};

class SensorDirecao{ //Classe do Sensor de Direção
public:
  SensorDirecao (int porta); //construtor da classe
  int LerAcX (void); //função de leitura do acelerômetro no eixo X
  int LerAcY (void); //função de leitura do acelerômetro no eixo Y
  int LerAcZ (void); //função de leitura do acelerômetro no eixo Z
  int LerGrX (void); //função de leitura do girômetro no eixo X
  int LerGrY (void); //função de leitura do girômetro no eixo Y
  int LerGrZ (void); //função de leitura do girômetro no eixo Z
private:
  int port; //variável para guardar a porta em que o sensor está conectado
};

#endif
