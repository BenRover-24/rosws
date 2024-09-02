#include <Servo.h> 

#define NUM_SERVOS 8
Servo servos[NUM_SERVOS];  

void setup() {
  Serial.begin(115200); 

  //  Attacher les servos aux pins 
  servos[0].attach(2); 
  servos[1].attach(3);
  servos[2].attach(4);
  servos[3].attach(5);
  servos[4].attach(6);
  servos[5].attach(7);
  servos[6].attach(8);
  servos[7].attach(9);

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Vérifier si la commande est pour les servos
    if (command.startsWith("S,")) {
      // Retirer le préfixe "S,"
      command = command.substring(2);  

      // Décoder les angles (format CSV)
      int angles[NUM_SERVOS];
      int index = 0;
      for (int i = 0; i < command.length(); i++) {
        if (command.charAt(i) == ',') {
          angles[index++] = command.substring(0, i).toInt(); 
          command = command.substring(i + 1); 
          i = 0;
        }
      }
      if (index < NUM_SERVOS) {
        angles[index] = command.toInt();
      }

      //  Positionner les servos 
      for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(angles[i]);  
      }
    }
  }
}