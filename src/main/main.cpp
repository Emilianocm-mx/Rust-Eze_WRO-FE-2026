#include <Arduino.h>
#include <ESP32servo.h>

#define LED_BUILTIN 15
#define STBY 23 //D5
#define PWMA 22 //D4
#define AIN1 21 //D3
#define AIN2 2 //D2
#define servoPin 4 //D4



void setup() {
Serial.begin(115200); // Initialize serial communication for debugging
  
  //Inicio drive
  pinMode(STBY, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);

  //LED built-in
  pinMode(LED_BUILTIN, OUTPUT);

  //PWM
  ledcAttach(PWMA, 1000, 8); //attach PWMA to channel 0, with a frequency of 1kHz and a resolution of 8 bits

  //Start driver
  digitalWrite(STBY, HIGH);
  
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup complete, waiting for serial input to start motor control...");
}

void loop() {



  if(Serial.available()>0)
  {
    Serial.println("Serial input received, starting motor control...");
    digitalWrite(LED_BUILTIN, HIGH);
    
    //Direction 1
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    Serial.println("Going forward...");


    //Speed up
    for (int motorUP = 0; motorUP <= 180; motorUP++)
    {
      ledcWrite(PWMA, motorUP);
      delay(10);
    }

    delay(5000); // Keep the motor running at full speed for 5 seconds
    
    //Speed down
    for (int motorDOWN = 180; motorDOWN >= 0; motorDOWN--)
    {
      ledcWrite(PWMA, motorDOWN);
      delay(10);
    }

    delay(500);

    //Direction 2
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    Serial.println("Going backward...");

    //Speed up
    for (int motorUP = 0; motorUP <= 180; motorUP++)
    {
      ledcWrite(PWMA, motorUP);
      delay(10);
    }

    delay(5000); // Keep the motor running at full speed for 5 seconds

    //Speed down
    for (int motorDOWN = 180; motorDOWN >= 0; motorDOWN--)
    {
      ledcWrite(PWMA, motorDOWN);
      delay(10);
    }
    Serial.println("Motor control cycle complete.");
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
    
  }
  
}
