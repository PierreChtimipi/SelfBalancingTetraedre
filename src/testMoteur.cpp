#include <Arduino.h>      // Obligatoire sous PlatformIO
#include <ESP32Servo.h>   // librairie pour contrôler les servos sur ESP32, compatible avec les ESC

Servo esc;  
const int ESC_PIN = 22;   // pin de signal

void setup() {
  Serial.begin(115200);
  
  // ------ Configuration pour l'ESP32 ------
  // Ces timers sont nécessaires pour la librairie ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50); // Fréquence standard de 50Hz
  // Attacher le moteur (Pin, Min Pulse 1000us, Max Pulse 2000us)
  esc.attach(ESC_PIN, 1000, 2000);

  Serial.println("--------------------------------");
  Serial.println("ESP32 Pret !");
  Serial.println(">> VEUILLEZ BRANCHER LA BATTERIE MAINTENANT <<");
  Serial.println(">> Ecoutez la musique du moteur... <<");

  // ARMEMENT : On envoie 0 gaz pour dire à l'ESC qu'on est prêt
  esc.writeMicroseconds(1000); 

  // On laisse 7 secondes pour brancher la LiPo tranquillement
  delay(7000); 
  
  Serial.println("C'est parti !");
}

void loop() {
  // accélération progressive
  Serial.println("Acceleration...");
  for (int speed = 1000; speed <= 1200; speed += 5) {
    esc.writeMicroseconds(speed);
    delay(20); // pause pour que la montée soit fluide
  }

  delay(1000); // On reste à cette vitesse 1 seconde

  // Décélération
  Serial.println("Freinage...");
  for (int speed = 1200; speed >= 1000; speed -= 5) {
    esc.writeMicroseconds(speed);
    delay(20);
  }

  Serial.println("Pause...");
  delay(2000); // Pause de 2 secondes avant de recommencer
}