#include <Arduino.h>
#include <ArduinoJson.h>
#include <Stepper.h>
#include <Wire.h>

#define role 3
const int stepsPerRevolution = 200;
// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

 StaticJsonDocument<200> doc;



void requestEvent();



void setup() {
  Wire.begin(8);               
  Wire.onRequest(requestEvent);
  Serial.begin(9600);
  // while (!Serial) continue;
  pinMode(role, OUTPUT);
   doc["hiz"] = 5;
  // doc["otSayisi"] = 62;
   doc["tarananSira"] = 1;
    serializeJson(doc, Serial);
   Serial.println();
myStepper.setSpeed(60);


}
byte commands = 0; // 00000001 = sol 00000010 = sag 00000100 = manuel 00001000 = basla 00010000 = git
void loop() {
  // not used in this example
  //serializeJson(doc, Serial);
   //Serial.println();
 //delay(100);
  commands = 0; 
if (Serial.available()) 
  {
    // Allocate the JSON document
    // This one must be bigger than for the sender because it must store the strings
    StaticJsonDocument<300> doc2;

    // Read the JSON document from the "link" serial port
    DeserializationError err = deserializeJson(doc2, Serial);

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

      if (doc2["sol"] == 1)
      {
        commands = commands | 1;
        //digitalWrite(solPin, HIGH);
      }

      
      if (doc2["sag"] == 1)
      {
       commands = commands | 2;
      }


      if (doc2["eksi"] == 1)
      {
        myStepper.step(stepsPerRevolution);
      }

      if (doc2["arti"] == 1)
      {
        myStepper.step(-stepsPerRevolution);
      }

      if (doc2["manuel"] == 1)
      {
       commands = commands | 4;
      }

      if (doc2["basla"] == 1)
      {
        commands = commands | 8;
      }

      if (doc2["git"] == 1)
      {
        commands = commands | 16;
      }

      
      
    } 
}}

void requestEvent() {
  Wire.write(commands);
}