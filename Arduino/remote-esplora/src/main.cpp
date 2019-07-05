
#include <Arduino.h>

// Inbuilt Libraries
#include <Esplora.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <TFT.h>

// External Libraries
#include "Plotter.h"

// Local Libraries
#include "SoftEasyTransfer.h"

// PINs
//#define CS   10
//#define DC   9
//#define RESET  8 

// Objects
SoftwareSerial bluetooth(3, 11);
Plotter p;
SoftEasyTransfer ET;
//TFT screen = TFT(CS, DC, RESET);

// Structs
struct SEND_DATA_STRUCTURE {
  int16_t speed;
};

// Variables
SEND_DATA_STRUCTURE dataRemoteToRobot;
int16_t speed;
char printBuffer[4];

// Program
void setup() {
    EsploraTFT.begin();  
    EsploraTFT.background(0,0,0);
    EsploraTFT.stroke(255,0,255);
    EsploraTFT.setTextSize(3);

    bluetooth.begin(9600);
    ET.begin(details(dataRemoteToRobot), &bluetooth);

	//p.Begin();
    //p.AddTimeGraph( "Some title of a graph", 1500, "label for x", x );
}

void loop() {
    int slider = Esplora.readSlider();
    int xAxis = Esplora.readAccelerometer(X_AXIS);
    int yAxis = Esplora.readAccelerometer(Y_AXIS);
    int zAxis = Esplora.readAccelerometer(Z_AXIS);
    bool btn1 = Esplora.readButton(SWITCH_1);
    bool btn2 = Esplora.readButton(SWITCH_2);
    bool btn3 = Esplora.readButton(SWITCH_3);
    bool btn4 = Esplora.readButton(SWITCH_4);
    int light = Esplora.readLightSensor();
    int temp = Esplora.readTemperature(DEGREES_C);
    int microphone = Esplora.readMicrophone();
    int joyBtn = Esplora.readJoystickSwitch();
    int joyX = Esplora.readJoystickX();
    int joyY = Esplora.readJoystickY();
    int red = 0;
    int green = 0;
    int blue = 0;
    int tone = 0;

    speed = slider;

    // Send to robot
    dataRemoteToRobot.speed = speed;
    ET.sendData();

    // Draw to screen
    EsploraTFT.stroke(0, 0, 0);
    EsploraTFT.text(printBuffer, 0, 10);

    String textSpeed = String(speed);
    textSpeed.toCharArray(printBuffer, 4);

    EsploraTFT.stroke(255, 255, 255);
    EsploraTFT.text(printBuffer, 0, 10);

    //x = 10*sin( 2.0*PI*( millis() / 5000.0 ) );

    //p.Plot(); // usually called within loop()

    // Esplora.writeRGB(255, 0, 0);  // make the LED red
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(0, 255, 0);  // make the LED green
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(0, 0, 255);  // make the LED blue
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(255, 255, 0); // make the LED yellow
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(0, 255, 255); // make the LED cyan
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(255, 0, 255); // make the LED magenta
    // delay(1000);                  // wait 1 second
    // Esplora.writeRGB(255, 255, 255); // make the LED white
    delay(200);                  // wait 1 second
}