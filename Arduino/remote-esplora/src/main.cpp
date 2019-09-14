
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

// Intevals
#define INTERVAL_TFT 200 // in ms
#define INTERVAL_SEND 50 // in ms
#define INTERVAL_READ_INPUT 100 // in ms

// TFT Paddings
#define PADDING_LEFT 30
#define PADDING_RIGHT 30
#define CHAR_WIDTH 6
#define COLUMN_WIDTH 20

// Objects
SoftwareSerial bluetooth(11, 3);
Plotter p;
SoftEasyTransfer ETin;
SoftEasyTransfer ETout;
//TFT screen = TFT(CS, DC, RESET);

// Enums
enum class AiAction {
    OFF = 0,
    IDLE = 1,
    PLANNING = 2,
    FORWARD = 3,
    BACKWARD = 4,
    LEFT = 5,
    RIGHT = 6,
    BRAKE = 7,
    STOP = 8,
    SLOW_FORWARD = 9,
    FAST_FORWARD = 10,
    SLOW_BACKWARD = 11,
    FAST_BACKWARD = 12,
    AVOID_LEFT = 13,
    AVOID_RIGHT = 14,
    BLOCKED = 15
};
enum class ModeControl {
    OPEN_LOOP = 0,
    CLOSED_LOOP = 1,
    TURRET = 2
};
enum class ModeAssist {
    OFF = 0,        // even in the OFF mode, robot will not collide with obstacles
    ADJUST = 1,     // robot is not moving on its own, but hwne it is moved by operator on a collision course, robot will adjust its direction (as long as it is accelerating)
    WANDER = 2,     // robot is moving on its own, wandering around (at any given moment of time, it can be disrupted/adjusted by operator, but even it this case, robot will not collide)
    ROAM = 3        // robot is roaming away/around, this is faster and more agressive than wander (at any given moment of time, it can be disrupted/adjusted by operator, but even it this case, robot will not collide)
};
enum class ModeEmergency {
    OK = 0,         // robot is OK, there is no emergency
    FULL_STOP = 1,  // robot is in full stop mode
    PID_RESET = 2   // robot is in full stop mode, PID loop is reset
};
enum class ModeLights {
    OFF = 0,        // lights are off
    AMBIENT = 1,    // lights are turned on in the desired brightness, colored based on movement direction (white in front, red in back)
    ADVANCED = 2,   // same as ambient with orange blinking when robot is turning
    FULL_ON = 3     // lights are full on (full white on all sides)
};

// Structs
struct SEND_DATA_STRUCTURE {
  int8_t mode_control;     // motor control mode, 0 = open loop, 1 = closed loop, 2 = turret control
  int8_t mode_assist;      // autopilot, 0 = off, 1 = adjust, 2 = wander, 3 = roam
  int8_t mode_emergency;   // 0 = OK, 1 = full_stop, 2 = full_stop + reset_pid
  int8_t mode_lights;      // 0 = OFF, 1 = ambient (white front, red back), 2 = advanced (default + directional blink), 3 = full ON (white everywhere)
  int8_t brightness;       // lights intensity, in percentage 0 - 100 (%)
  int8_t speed;            // speed, in percentage 0 - 100 (%)
  int8_t x;                // joystick X axis, in percentage -100 - 100 (%)
  int8_t y;                // joystick Y axis, in percentage -100 - 100 (%)
  int8_t acceleration;     // maximum PID loop acceleration configuration, in percentage 0 - 100 (%)
};
struct RECEIVE_DATA_STRUCTURE {
  int8_t max_speed;        // maximum PID loop speed, in percentage 0 - 100 (%) (computed based on the speed control)
  int8_t max_acceleration; // maximum PID loop acceleration, in percentage 0 - 100 (%)
  int8_t action;           // current robot's AI action: 0 = OFF, 1 = Idle, 2 = Planning, 3 = Forward, 4 = Backward, 5 = Left, 6 = Right, 7 = Brake, 8 = Stop,
                            // 9 = Slow Forward, 10 = Fast Forward, 11 = Slow Backward, 12 = Fast Backward,
                            // 13 = Avoid Left, 14 = Avoid Right, 15 = Blocked
  int8_t battery;          // battery, in percentage 0 - 100 (%)
  int8_t motor_right;      // power to right motor, in percentage 0 - 100 (%)
  int8_t motor_left;       // power to left motor, in percentage 0 - 100 (%)
  int8_t distance_fr;      // distance Front-Right, in cm, 9-81cm
  int8_t distance_fl;      // distance Front-Left, in cm, 9-81cm
  int8_t distance_br;      // distance Back-Right, in cm, 9-81cm
  int8_t distance_bl;      // distance Back-Left, in cm, 9-81cm
};

// Variables
RECEIVE_DATA_STRUCTURE dataRobotToRemote;
SEND_DATA_STRUCTURE dataRemoteToRobot;

unsigned long tftMillis = 0;
unsigned long sendMillis = 0;
unsigned long readInputMillis = 0;

// Compute
unsigned int light;
int temperature;
int16_t luminosity;
int16_t limitedSpeed;       // 0-950
int16_t rawSpeed;           // 0-1024
int16_t rawX;               // -512-512
int16_t rawY;               // -512-512

// Out
int8_t mode_control;
int8_t mode_assist;
int8_t mode_emergency;
int8_t mode_lights;
int8_t brightness;
int8_t speed;
int8_t x;
int8_t y;
int8_t acceleration;

// In
int8_t max_speed;           // maximum PID loop speed, in percentage 0 - 100 (%) (computed based on the speed control)
int8_t max_acceleration;    // maximum PID loop acceleration, in percentage 0 - 100 (%)
int8_t action;              // current robot's AI action: 0 = OFF, 1 = Idle, 2 = Planning, 3 = Forward, 4 = Backward, 5 = Left, 6 = Right, 7 = Brake, 8 = Stop,
                            // 9 = Slow Forward, 10 = Fast Forward, 11 = Slow Backward, 12 = Fast Backward,
                            // 13 = Avoid Left, 14 = Avoid Right, 15 = Blocked
int8_t battery;             // battery, in percentage 0 - 100 (%)
int8_t motor_right;         // power to right motor, in percentage 0 - 100 (%)
int8_t motor_left;          // power to left motor, in percentage 0 - 100 (%)
int8_t distance_fr;         // distance Front-Right, in cm, 9-81cm
int8_t distance_fl;         // distance Front-Left, in cm, 9-81cm
int8_t distance_br;         // distance Back-Right, in cm, 9-81cm
int8_t distance_bl;         // distance Back-Left, in cm, 9-81cm

char printSpeedBuffer[4] = "";
char printAccelerationBuffer[4] = "";
char printActionBuffer[7] = "";
char printBatteryBuffer[4] = "";
char printTemperatureBuffer[4] = "";
char printBrightnessBuffer[4] = "";
char printLuminosityBuffer[4] = "";
char printModeControl[4] = "";
char printModeAssist[4] = "";
char printModeEmergency[4] = "";
char printModeLights[4] = "";

// Functions
String labelAction(int8_t action) {
    switch (action) {
        case static_cast<int8_t>(AiAction::OFF):
            return "OFF    ";
            break;
        case static_cast<int8_t>(AiAction::IDLE):
            return "IDLE   ";
            break;
        case static_cast<int8_t>(AiAction::PLANNING):
            return "PLAN   ";
            break;
        case static_cast<int8_t>(AiAction::FORWARD):
            return ">>     ";
            break;
        case static_cast<int8_t>(AiAction::BACKWARD):
            return "<<     ";
            break;
        case static_cast<int8_t>(AiAction::LEFT):
            return "LEFT   ";
            break;
        case static_cast<int8_t>(AiAction::RIGHT):
            return "RIGHT  ";
            break;
        case static_cast<int8_t>(AiAction::BRAKE):
            return "BRAKE  ";
            break;
        case static_cast<int8_t>(AiAction::STOP):
            return "STOP   ";
            break;
        case static_cast<int8_t>(AiAction::SLOW_FORWARD):
            return ">      ";
            break;
        case static_cast<int8_t>(AiAction::FAST_FORWARD):
            return ">>>    ";
            break;
        case static_cast<int8_t>(AiAction::SLOW_BACKWARD):
            return "<      ";
            break;
        case static_cast<int8_t>(AiAction::FAST_BACKWARD):
            return "<<<    ";
            break;
        case static_cast<int8_t>(AiAction::AVOID_LEFT):
            return "AVOID L";
            break;
        case static_cast<int8_t>(AiAction::AVOID_RIGHT):
            return "AVOID R";
            break;
        case static_cast<int8_t>(AiAction::BLOCKED):
            return "BLOCKED";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

String labelModeControl(int8_t mode_control) {
    switch (mode_control) {
        case static_cast<int8_t>(ModeControl::OPEN_LOOP):
            return "OL";
            break;
        case static_cast<int8_t>(ModeControl::CLOSED_LOOP):
            return "CL";
            break;
        case static_cast<int8_t>(ModeControl::TURRET):
            return "TU";
            break;
        default:
            return "--";
            break;
    }
}

String labelModeAssist(int8_t mode_assist) {
    switch (mode_assist) {
        case static_cast<int8_t>(ModeAssist::OFF):
            return "OFF";
            break;
        case static_cast<int8_t>(ModeAssist::ADJUST):
            return "ADJ";
            break;
        case static_cast<int8_t>(ModeAssist::WANDER):
            return "WAN";
            break;
        case static_cast<int8_t>(ModeAssist::ROAM):
            return "ROA";
            break;
        default:
            return " - ";
            break;
    }
}

String labelModeEmergency(int8_t mode_emergency) {
    switch (mode_emergency) {
        case static_cast<int8_t>(ModeEmergency::OK):
            return " OK ";
            break;
        case static_cast<int8_t>(ModeEmergency::FULL_STOP):
            return "STOP";
            break;
        case static_cast<int8_t>(ModeEmergency::PID_RESET):
            return "REST";
            break;
        default:
            return " -- ";
            break;
    }
}

String labelModeLights(int8_t mode_lights) {
    switch (mode_lights) {
        case static_cast<int8_t>(ModeLights::OFF):
            return "OFF";
            break;
        case static_cast<int8_t>(ModeLights::AMBIENT):
            return "AMB";
            break;
        case static_cast<int8_t>(ModeLights::ADVANCED):
            return "ADV";
            break;
        case static_cast<int8_t>(ModeLights::FULL_ON):
            return "FLL";
            break;
        default:
            return " - ";
            break;
    }
}

int8_t adjustModeControl(int8_t mode) {
    int8_t new_mode = mode + 1;
    if (new_mode > 2) { new_mode = 0; }
    return new_mode;
}

int8_t adjustModeAssist(int8_t mode) {
    int8_t new_mode = mode + 1;
    if (new_mode > 3) { new_mode = 0; }
    return new_mode;
}

int8_t adjustModeEmergency(int8_t mode) {
    int8_t new_mode = mode + 1;
    if (new_mode > 2) { new_mode = 0; }
    return new_mode;
}

int8_t adjustModeLights(int8_t mode) {
    int8_t new_mode = mode + 1;
    if (new_mode > 3) { new_mode = 0; }
    return new_mode;
}

#define INDICATOR_LEFT 0
#define INDICATOR_RIGHT 1
#define INDICATOR_DIRECTIONAL_DECREMENT_STEP 1
// 0 - 100
#define DISTANCE_LEVEL_1 15
#define DISTANCE_LEVEL_2 30
#define DISTANCE_LEVEL_3 40
#define DISTANCE_LEVEL_4 45
#define DISTANCE_LEVEL_5 50
void drawTopDistanceIndicator(int8_t distance, int x, int y, int direction /* 0 = Left, 1 = Right */, int width = 20, int max_height = 60) {
    int gap_interval = 5;   // in px
    int box_size = 5;       // in px, including gaps
    int height = map(distance, 0, 100, 0, max_height);

    uint16_t color = ST7735_WHITE;

    int directional_width = width;
    int directional_x = x;
    int max_box_amount = height / box_size;
    for (int i = 0; i < (max_height / box_size); i++) {
        int box_start = (i * box_size);

        if (box_start <= DISTANCE_LEVEL_1) {
            color = ST7735_GREEN;
        } else if (box_start > DISTANCE_LEVEL_1 && box_start <= DISTANCE_LEVEL_2) {
            color = EsploraTFT.Color565(187, 255, 59); // light green, #BBFF3B
        } else if (box_start > DISTANCE_LEVEL_2 && box_start <= DISTANCE_LEVEL_3) {
            color = EsploraTFT.Color565(255, 227, 43); // green-yellow, #FFE32B
        } else if (box_start > DISTANCE_LEVEL_3 && box_start <= DISTANCE_LEVEL_4) {
            color = EsploraTFT.Color565(255, 182, 56); // orange, #FFB638
        } else if (box_start > DISTANCE_LEVEL_4 && box_start <= DISTANCE_LEVEL_5) {
            color = EsploraTFT.Color565(255, 111, 43); // orange-red, #FF6F2B
        } else {
            color = ST7735_RED;
        }

        if (i >= max_box_amount) {
            color = ST7735_BLACK;
        }

        if (direction == INDICATOR_LEFT) {
            directional_width -= INDICATOR_DIRECTIONAL_DECREMENT_STEP;
        } else { // INDICATOR_RIGHT
            directional_width -= INDICATOR_DIRECTIONAL_DECREMENT_STEP;
            directional_x += INDICATOR_DIRECTIONAL_DECREMENT_STEP;
        }

        EsploraTFT.fillRect(directional_x, y + box_start, directional_width, box_size, color);
    }

    // Draw gaps
    for (int i = 0; i < (max_height / gap_interval); i++) {
        int gap_start = (i * gap_interval);
        EsploraTFT.fillRect(x, y + gap_start, width, 1, ST7735_BLACK);
    }

    // Draw black proportional part
    //EsploraTFT.fillRect(x, height, width, (max_height - height), ST7735_BLACK);
}

void drawBottomDistanceIndicator(int8_t distance, int x, int y, int direction /* 0 = Left, 1 = Right */, int width = 20, int max_height = 60) {
    int gap_interval = 5;   // in px
    int box_size = 5;       // in px, including gaps
    int height = map(distance, 0, 100, 0, max_height);

    uint16_t color = ST7735_WHITE;

    int directional_width = width;
    int directional_x = x;
    int max_box_amount = height / box_size;
    for (int i = 0; i < (max_height / box_size); i++) {
        int box_start = (i * box_size);

        if (box_start <= DISTANCE_LEVEL_1) {
            color = ST7735_GREEN;
        } else if (box_start > DISTANCE_LEVEL_1 && box_start <= DISTANCE_LEVEL_2) {
            color = EsploraTFT.Color565(187, 255, 59); // light green, #BBFF3B
        } else if (box_start > DISTANCE_LEVEL_2 && box_start <= DISTANCE_LEVEL_3) {
            color = EsploraTFT.Color565(255, 227, 43); // green-yellow, #FFE32B
        } else if (box_start > DISTANCE_LEVEL_3 && box_start <= DISTANCE_LEVEL_4) {
            color = EsploraTFT.Color565(255, 182, 56); // orange, #FFB638
        } else if (box_start > DISTANCE_LEVEL_4 && box_start <= DISTANCE_LEVEL_5) {
            color = EsploraTFT.Color565(255, 111, 43); // orange-red, #FF6F2B
        } else {
            color = ST7735_RED;
        }

        if (i >= max_box_amount) {
            color = ST7735_BLACK;
        }

        if (direction == INDICATOR_LEFT) {
            directional_width -= INDICATOR_DIRECTIONAL_DECREMENT_STEP;
        } else { // INDICATOR_RIGHT
            directional_width -= INDICATOR_DIRECTIONAL_DECREMENT_STEP;
            directional_x += INDICATOR_DIRECTIONAL_DECREMENT_STEP;
        }

        EsploraTFT.fillRect(directional_x, y + ((max_height - box_size) - box_start), directional_width, box_size, color);
    }

    // Draw gaps
    for (int i = 0; i < (max_height / gap_interval); i++) {
        int gap_start = (i * gap_interval);
        EsploraTFT.fillRect(x, y + ((max_height - box_size) - gap_start), width, 1, ST7735_BLACK);
    }

    // Draw black proportional part
    //EsploraTFT.fillRect(x, y, width, (max_height - height), ST7735_BLACK);
}

#define SPEED_LEVEL_1 10
#define SPEED_LEVEL_2 15
#define SPEED_LEVEL_3 40
#define SPEED_LEVEL_4 45
#define SPEED_LEVEL_5 50
void drawMotorIndicator(int8_t motor_power, int x, int y, int width = 20, int max_height = 50) {
    int height = map(motor_power, 0, 100, 0, max_height);

    uint16_t color = ST7735_WHITE;
    if (height < SPEED_LEVEL_1) {
        color = ST7735_MAGENTA;
    } else if (height > SPEED_LEVEL_1 && height <= SPEED_LEVEL_2) {
        color = ST7735_CYAN;
    } else if (height > SPEED_LEVEL_2 && height <= SPEED_LEVEL_3) {
        color = ST7735_GREEN;
    } else if (height > SPEED_LEVEL_3 && height <= SPEED_LEVEL_4) {
        color = ST7735_YELLOW;
    } else if (height > SPEED_LEVEL_4 && height <= SPEED_LEVEL_5) {
        color = ST7735_RED;
    } else {
        color = ST7735_WHITE;
    }

    EsploraTFT.fillRect(x, y, width, (max_height - height), ST7735_BLACK);
    EsploraTFT.fillRect(x, y + (max_height - height), width, height, color);
}

void drawMotorMaxSpeedIndicator(int8_t max_speed, int x, int y, int width = 1, int max_height = 50) {
    int height = map(max_speed, 0, 100, 0, max_height);

    EsploraTFT.fillRect(x, y, width, (max_height - height), ST7735_BLACK);
    EsploraTFT.fillRect(x, y + (max_height - height), width, height, ST7735_WHITE);
}

// int distance; // temp
// bool distance_dir; // temp

// Program
void setup() {
    EsploraTFT.begin();
    EsploraTFT.background(0,0,0);
    EsploraTFT.setTextSize(1);

    Esplora.writeRGB(30, 0, 0);

    //Serial.begin(9600);
    //while (!Serial) { ; }
    //Serial.println("Rowee Esplora Remote is ready");

    mode_control = static_cast<int8_t>(ModeControl::OPEN_LOOP);
    mode_assist = static_cast<int8_t>(ModeAssist::OFF);
    mode_emergency = static_cast<int8_t>(ModeEmergency::OK);
    mode_lights = static_cast<int8_t>(ModeLights::OFF);

    bluetooth.begin(9600);
    while (!bluetooth) { ; }
    ETin.begin(details(dataRobotToRemote), &bluetooth);
    ETout.begin(details(dataRemoteToRobot), &bluetooth);

	//p.Begin();
    //p.AddTimeGraph( "Some title of a graph", 1500, "label for x", x );

    // Print fixed values to TFT
    EsploraTFT.stroke(255, 255, 255);
    EsploraTFT.text("BAT    %", PADDING_LEFT + 0, 0);
    EsploraTFT.text("TMP", PADDING_LEFT + (CHAR_WIDTH * 9), 0);
    EsploraTFT.drawCircle(PADDING_LEFT + (CHAR_WIDTH * 15) + 3, 1, 1, ST7735_WHITE);
    EsploraTFT.text("C", PADDING_LEFT + (CHAR_WIDTH * 16), 0);
    EsploraTFT.text("SPD    %", PADDING_LEFT + 0, 10);
    EsploraTFT.text("ACL    %", PADDING_LEFT + (CHAR_WIDTH * 9), 10);
    EsploraTFT.text("LUM    %", PADDING_LEFT + 0, 20);
    EsploraTFT.text("LHT    %", PADDING_LEFT + (CHAR_WIDTH * 9), 20);
    EsploraTFT.text("L", PADDING_LEFT + CHAR_WIDTH + 2, 68);
    EsploraTFT.text("R", PADDING_LEFT + CHAR_WIDTH + 2 + 80, 68);

    EsploraTFT.stroke(200, 200, 200);
    EsploraTFT.text("Mode", PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 13, 68);
    EsploraTFT.text("Lht", PADDING_LEFT + COLUMN_WIDTH - (CHAR_WIDTH * 2) + 14, 88);
    EsploraTFT.text("AI", PADDING_LEFT + COLUMN_WIDTH + (CHAR_WIDTH * 6) + 12, 88);
    EsploraTFT.text("Emrg", PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 13, 118);

    EsploraTFT.stroke(255, 255, 255);
    EsploraTFT.drawCircle(PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 24, 75 + 12 + 6, 2, ST7735_WHITE);
    EsploraTFT.drawCircle(PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 24 - 5, 75 + 17 + 6, 2, ST7735_WHITE);
    EsploraTFT.drawCircle(PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 24 + 5, 75 + 17 + 6, 2, ST7735_WHITE);
    EsploraTFT.drawCircle(PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 24, 75 + 22 + 6, 2, ST7735_WHITE);

    Esplora.writeRGB(0, 0, 30);

    // distance = 0;
    // distance_dir = false;
}

void loop() {
    unsigned long currentMillis = millis();

    // Load data from Robot
    if (ETin.receiveData()) {
        max_speed = dataRobotToRemote.max_speed;                    // maximum PID loop speed, in percentage 0 - 100 (%) (computed based on the speed control)
        max_acceleration = dataRobotToRemote.max_acceleration;      // maximum PID loop acceleration, in percentage 0 - 100 (%)
        action = dataRobotToRemote.action;                          // current robot's AI action
        battery = dataRobotToRemote.battery;                        // battery, in percentage 0 - 100 (%)
        motor_right = dataRobotToRemote.motor_right;                // power to right motor, in percentage 0 - 100 (%)
        motor_left = dataRobotToRemote.motor_left;                  // power to left motor, in percentage 0 - 100 (%)
        distance_fr = dataRobotToRemote.distance_fr;                // distance Front-Right, in cm, 9-81cm
        distance_fl = dataRobotToRemote.distance_fl;                // distance Front-Left, in cm, 9-81cm
        distance_br = dataRobotToRemote.distance_br;                // distance Back-Right, in cm, 9-81cm
        distance_bl = dataRobotToRemote.distance_bl;                // distance Back-Left, in cm, 9-81cm
    }

    if (battery == 0) {
        Esplora.writeRGB(30, 0, 0);
    } else {
        Esplora.writeRGB(0, 30, 0);
    }

    // Load data from input controls
    //int xAxis = Esplora.readAccelerometer(X_AXIS);
    //int yAxis = Esplora.readAccelerometer(Y_AXIS);
    //int zAxis = Esplora.readAccelerometer(Z_AXIS);
    light = Esplora.readLightSensor();
    temperature = Esplora.readTemperature(DEGREES_C);
    //int microphone = Esplora.readMicrophone();
    int joyBtn = Esplora.readJoystickSwitch();
    rawSpeed = Esplora.readSlider();
    rawX = Esplora.readJoystickX() - 20;
    rawY = Esplora.readJoystickY() - 5;
    //int tone = 0;

    if (currentMillis - readInputMillis >= INTERVAL_READ_INPUT) {
        readInputMillis = currentMillis;

        bool btnEmergency = Esplora.readButton(SWITCH_1);
        bool btnLights = Esplora.readButton(SWITCH_2);
        bool btnControlMode = Esplora.readButton(SWITCH_3);
        bool btnAssistMode = Esplora.readButton(SWITCH_4);

        if (btnEmergency == LOW) { mode_emergency = adjustModeEmergency(mode_emergency); }
        if (btnLights == LOW) { mode_lights = adjustModeLights(mode_lights); }
        if (btnControlMode == LOW) { mode_control = adjustModeControl(mode_control); }
        if (btnAssistMode == LOW) { mode_assist = adjustModeAssist(mode_assist); }
    }

    // Compute data
    luminosity = map(light, 0, 1023, 0, 100);
    brightness = 100 - luminosity;
    acceleration = 100; // TODO load from settings
    limitedSpeed = 950 - min(rawSpeed, 950);
    speed = map(limitedSpeed, 1, 950, 30, 100);
    x = map(rawX, -532, 491, -100, 100);
    y = map(rawY, -517, 506, -100, 100);

    // Fake distance generator
    // if (distance_dir == false) {
    //     distance += 1;
    // } else {
    //     distance -= 1;
    // }

    // if (distance >= 100) {
    //     distance = 100;
    //     distance_dir = true;
    // }
    // if (distance <= 0) {
    //     distance = 0;
    //     distance_dir = false;
    // }
    //distance = 50;

    // Display data
    if (currentMillis - tftMillis >= INTERVAL_TFT) {
        tftMillis = currentMillis;

        EsploraTFT.setTextSize(1);

        // # Battery
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printBatteryBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 0);
        // Get new value
        String textBattery = String(battery);
        textBattery.toCharArray(printBatteryBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printBatteryBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 0);

        // # Temperature
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printTemperatureBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 0);
        // Get new value
        String textTemperature = String(temperature);
        textTemperature.toCharArray(printTemperatureBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printTemperatureBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 0);

        // # Speed
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printSpeedBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 10);
        // Get new value
        String textSpeed = String(max_speed);
        textSpeed.toCharArray(printSpeedBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printSpeedBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 10);

        // # Acceleration
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printAccelerationBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 10);
        // Get new value
        String textAcceleration = String(max_acceleration);
        textAcceleration.toCharArray(printAccelerationBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printAccelerationBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 10);

        // # Luminosity
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printLuminosityBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 20);
        // Get new value
        String textLuminosity = String(luminosity);
        textLuminosity.toCharArray(printLuminosityBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printLuminosityBuffer, PADDING_LEFT + (CHAR_WIDTH * 4), 20);

        // # Brightness
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printBrightnessBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 20);
        // Get new value
        String textBrightness = String(brightness);
        textBrightness.toCharArray(printBrightnessBuffer, 4);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printBrightnessBuffer, PADDING_LEFT + (CHAR_WIDTH * 13), 20);

        // # Action
        EsploraTFT.setTextSize(2);
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printActionBuffer, PADDING_LEFT + 0, 40);
        // Get new value
        String textAction = labelAction(action);
        textAction.toCharArray(printActionBuffer, 7);
        // Print new value
        EsploraTFT.stroke(255, 255, 255);
        EsploraTFT.text(printActionBuffer, PADDING_LEFT + 0, 40);
        EsploraTFT.setTextSize(1);

        // Modes

        // - Mode Control
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printModeControl, PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + (CHAR_WIDTH * 1) + 13, 78);
        // Get new value
        String textModeControl = labelModeControl(mode_control);
        textModeControl.toCharArray(printModeControl, 3);
        // Print new value
        if (mode_control == static_cast<int8_t>(ModeControl::OPEN_LOOP)) {
            EsploraTFT.stroke(255, 227, 43);
        } else if (mode_control == static_cast<int8_t>(ModeControl::CLOSED_LOOP)) {
            EsploraTFT.stroke(0, 255, 0);
        } else { // TURRET
            EsploraTFT.stroke(41, 234, 255);
        }
        EsploraTFT.text(printModeControl, PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + (CHAR_WIDTH * 1) + 13, 78);

        // - Mode Lights
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printModeLights, PADDING_LEFT + COLUMN_WIDTH - (CHAR_WIDTH * 2) + 14, 98);
        // Get new value
        String textModeLights = labelModeLights(mode_lights);
        textModeLights.toCharArray(printModeLights, 4);
        // Print new value
        if (mode_lights == static_cast<int8_t>(ModeLights::OFF)) {
            EsploraTFT.stroke(180, 180, 180);
        } else if (mode_lights == static_cast<int8_t>(ModeLights::AMBIENT)) {
            EsploraTFT.stroke(41, 234, 255);
        } else if (mode_lights == static_cast<int8_t>(ModeLights::ADVANCED)) {
            EsploraTFT.stroke(0, 255, 0);
        } else { // FULL_ON
            EsploraTFT.stroke(255, 227, 43);
        }
        EsploraTFT.text(printModeLights, PADDING_LEFT + COLUMN_WIDTH - (CHAR_WIDTH * 2) + 14, 98);

        // - Mode Assist
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printModeAssist, PADDING_LEFT + COLUMN_WIDTH + (CHAR_WIDTH * 5) + 11, 98);
        // Get new value
        String textModeAssist = labelModeAssist(mode_assist);
        textModeAssist.toCharArray(printModeAssist, 4);
        // Print new value
        if (mode_assist == static_cast<int8_t>(ModeAssist::OFF)) {
            EsploraTFT.stroke(180, 180, 180);
        } else if (mode_assist == static_cast<int8_t>(ModeAssist::ADJUST)) {
            EsploraTFT.stroke(0, 255, 0);
        } else if (mode_assist == static_cast<int8_t>(ModeAssist::WANDER)) {
            EsploraTFT.stroke(41, 234, 255);
        } else { // ROAM
            EsploraTFT.stroke(255, 227, 43);
        }
        EsploraTFT.text(printModeAssist, PADDING_LEFT + COLUMN_WIDTH + (CHAR_WIDTH * 5) + 11, 98);

        // - Mode Emergency
        // Clear old value
        EsploraTFT.stroke(0, 0, 0);
        EsploraTFT.text(printModeEmergency, PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 13, 108);
        // Get new value
        String textModeEmergency = labelModeEmergency(mode_emergency);
        textModeEmergency.toCharArray(printModeEmergency, 5);
        // Print new value
        if (mode_emergency == static_cast<int8_t>(ModeEmergency::OK)) {
            EsploraTFT.stroke(0, 255, 0);
        } else {
            EsploraTFT.stroke(255, 0, 0);
        }
        EsploraTFT.text(printModeEmergency, PADDING_LEFT + COLUMN_WIDTH + CHAR_WIDTH + 13, 108);

        // # Distance
        drawTopDistanceIndicator(100 - map(distance_fl, 9, 81, 0, 100), 5, 0, INDICATOR_LEFT);
        drawTopDistanceIndicator(100 - map(distance_fr, 9, 81, 0, 100), 160 - 25, 0, INDICATOR_RIGHT);
        drawBottomDistanceIndicator(100 - map(distance_bl, 9, 81, 0, 100), 5, 128 - 60, INDICATOR_LEFT);
        drawBottomDistanceIndicator(100 - map(distance_br, 9, 81, 0, 100), 160 - 25, 128 - 60, INDICATOR_RIGHT);

        // # Motors max speed
        drawMotorMaxSpeedIndicator(speed, PADDING_LEFT + 20, 128 - 50);
        drawMotorMaxSpeedIndicator(speed, PADDING_LEFT + 20 + 59, 128 - 50);

        // # Motors
        drawMotorIndicator(motor_left, PADDING_LEFT + 0, 128 - 50);
        drawMotorIndicator(motor_right, PADDING_LEFT + 80, 128 - 50);
    }

    // Send data to Robot
    if (currentMillis - sendMillis >= INTERVAL_SEND) {
        sendMillis = currentMillis;

        dataRemoteToRobot.mode_control = mode_control;
        dataRemoteToRobot.mode_assist = mode_assist;
        dataRemoteToRobot.mode_emergency = mode_emergency;
        dataRemoteToRobot.mode_lights = mode_lights;
        dataRemoteToRobot.brightness = brightness;
        dataRemoteToRobot.speed = speed;
        dataRemoteToRobot.x = x;
        dataRemoteToRobot.y = y;
        dataRemoteToRobot.acceleration = acceleration;

        ETout.sendData();
    }

    //p.Plot(); // usually called within loop()
}