#include <Arduino.h>
#include <ArduinoJson.h>
#include <Stepper.h>
#define role 2

const int stepsPerRevolution = 200;
// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

 StaticJsonDocument<200> doc;
/* int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;*/
void setup() {
  Serial.begin(9600);
  // while (!Serial) continue;
  //  pinMode(role, OUTPUT);
   doc["hiz"] = 5;
  // doc["otSayisi"] = 62;
   doc["tarananSira"] = 1;
    serializeJson(doc, Serial);
   Serial.println();
myStepper.setSpeed(60);
  /*pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);*/
}

void loop() {
  // not used in this example
  //serializeJson(doc, Serial);
   //Serial.println();
 //delay(100);
  
if (Serial.available()) 
  {
    // Allocate the JSON document
    // This one must be bigger than for the sender because it must store the strings
    StaticJsonDocument<300> doc2;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc2, Serial);
       digitalWrite(role, HIGH);

    if (err == DeserializationError::Ok) 
    {
      // Print the values
      // (we must use as<T>() to resolve the ambiguity)
       
      if (doc2["spray"] == 1)
      {
        digitalWrite(role, HIGH);
      }else{
        digitalWrite(role, LOW);
      }
      delay(300);

      /*if (doc2["sol"] == 1)
      {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      // set speed to 200 out of possible range 0~255
        analogWrite(enA, 200);
      }else{
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW); 
      }

      
      if (doc2["sag"] == 1)
      {
        digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
      // set speed to 200 out of possible range 0~255
        analogWrite(enA, 200);
      }else{
          digitalWrite(in3, LOW);
          digitalWrite(in4, LOW); 
      }
*/
      if (doc2["eksi"] == 1)
      {
        myStepper.step(stepsPerRevolution);
      }

      if (doc2["artı"] == 1)
      {
        myStepper.step(-stepsPerRevolution);
      }
      
      
    } 
}}
