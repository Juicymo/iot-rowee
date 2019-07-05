
#include <Arduino.h>

// Inbuilt Libraries
//#include <Servo.h>
//#include <Wire.h>

// External Libraries
#include "Plotter.h"

// Local Libraries
#include "EasyTransfer.h"

// Objects
Plotter p;
EasyTransfer ET;

// Structs
struct RECEIVE_DATA_STRUCTURE {
  int16_t speed;
};

// Variables
RECEIVE_DATA_STRUCTURE dataRemoteToRobot;
int16_t speed;

// Program
void setup() {
    Serial2.begin(9600);
    ET.begin(details(dataRemoteToRobot), &Serial2);

	p.Begin();
    p.AddTimeGraph("Rowee", 1500, "speed", speed);
}

void loop() {
    if (ET.receiveData()) {
        speed = dataRemoteToRobot.speed;
    }

    p.Plot();

    delay(100);
}