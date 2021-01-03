/*
 * Código exemplo do robô CREPE
 * O objetivo deste código é caso o robô se choque com um objeto, ele anda um pouco para trás e gira para a esquerda, depois continua andando para frente.
 * Criado por: Alexandre Crepory
 */

#include <Crepe.h>

Robo robo; 
SensorToque toque (2); 

void setup() {
  }

void loop() {
  robo.IrFrente(60);
  delay(25);
  if((toque.LerA() == 1)||(toque.LerB() == 1)){
    robo.Parar();
    delay(50);
    robo.IrTras(50);
    delay(300);
    robo.GirarEsquerda(75);
    delay(500);
  }

}
