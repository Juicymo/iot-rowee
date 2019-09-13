
#include <Arduino.h>

// Inbuilt Libraries
//#include <Servo.h>
#include <Wire.h>

// External Libraries
#include "PWMServo.h"
#include "Plotter.h"
#include "LiquidCrystal.h"
//#include "SharpIR.h"
#include "Adafruit_NeoPixel.h"

// Local Libraries
#include "EasyTransfer.h"
#include "RoboClaw.h"

// Local Files
#include "communications.h"
#include "differential_steering.h"
//#include "hal_roboclaw.h"
#include "open_loop_control.h"
#include "closed_loop_control.h"
//#include "hal_lcd.h"
//#include "hal_sensors_infrared_distance.h"
//#include "hal_lights.h"
//#include "debug_plotter.h"

// Constants
#define TIMEOUT_ROBOCLAW 10000 // in ms
#define SERVO_PAN_CENTER 102
#define SERVO_TILT_CENTER 84
#define MOTOR_DIRECTION_FORWARD 0
#define MOTOR_DIRECTION_BACKWARD 1

// Intevals
#define INTERVAL_LCD 500 // in ms
#define INTERVAL_SEND 100 // in ms

// Pins - LCD
#define PIN_LCD_RS 12
#define PIN_LCD_EN 11
#define PIN_LCD_D4 24
#define PIN_LCD_D5 25
#define PIN_LCD_D6 26
#define PIN_LCD_D7 27
#define PIN_SERVO_PAN 29
#define PIN_SERVO_TILT 30

// Objects
Plotter p;
EasyTransfer ETin;
EasyTransfer ETout;
RoboClaw roboclaw(&Serial1, TIMEOUT_ROBOCLAW);
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
PWMServo servoPan;     // left-right movement
PWMServo servoTilt;    // top-down movement
//SharpIR sensorFR(SharpIR::GP2Y0A41SK0F, A0);

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
struct RECEIVE_DATA_STRUCTURE {
  int16_t mode_control;     // motor control mode, 0 = open loop, 1 = closed loop, 2 = turret control
  int16_t mode_assist;      // autopilot, 0 = off, 1 = adjust, 2 = wander, 3 = roam
  int16_t mode_emergency;   // 0 = OK, 1 = full_stop, 2 = full_stop + reset_pid
  int16_t mode_lights;      // 0 = OFF, 1 = ambient (white front, red back), 2 = advanced (default + directional blink), 3 = full ON (white everywhere)
  int16_t brightness;       // lights intensity, in percentage 0 - 100 (%)
  int16_t speed;            // speed, raw data from controls
  int16_t x;                // joystick X axis, raw data from controls
  int16_t y;                // joystick Y axis, raw data from controls
  int16_t acceleration;     // maximum PID loop acceleration configuration, in percentage 0 - 100 (%)
};
struct SEND_DATA_STRUCTURE {
  int16_t max_speed;        // maximum PID loop speed, in percentage 0 - 100 (%) (computed based on the speed control)
  int16_t max_acceleration; // maximum PID loop acceleration, in percentage 0 - 100 (%)
  int16_t action;           // current robot's AI action: 0 = OFF, 1 = Idle, 2 = Planning, 3 = Forward, 4 = Backward, 5 = Left, 6 = Right, 7 = Brake, 8 = Stop,
                            // 9 = Slow Forward, 10 = Fast Forward, 11 = Slow Backward, 12 = Fast Backward,
                            // 13 = Avoid Left, 14 = Avoid Right, 15 = Blocked
  int16_t battery;          // battery, in percentage 0 - 100 (%)
  int16_t motor_right;      // power to right motor, in percentage 0 - 100 (%)
  int16_t motor_left;       // power to left motor, in percentage 0 - 100 (%)
  int16_t distance_fr;      // distance Front-Right, in cm
  int16_t distance_fl;      // distance Front-Left, in cm
  int16_t distance_br;      // distance Back-Right, in cm
  int16_t distance_bl;      // distance Back-Left, in cm
};

// Variables
RECEIVE_DATA_STRUCTURE dataRemoteToRobot;
SEND_DATA_STRUCTURE dataRobotToRemote;

unsigned long lcdMillis = 0;
unsigned long sendMillis = 0;

int16_t speed;
int16_t limitedSpeed;
int16_t normSpeed;
int16_t percentSpeed;
double speedInPercent;
int16_t x;
int16_t y;
int absMotorL;
int absMotorR;

int16_t currentR;
int16_t currentL;
uint16_t voltage;
uint16_t temp;
uint32_t speedR;
uint32_t speedL;
uint32_t prevSpeedR;
uint32_t prevSpeedL;

double speedInPercentR;
double speedInPercentL;
uint32_t maxSpeedR;
uint32_t maxSpeedL;
uint32_t scaledSpeedR;
uint32_t scaledSpeedL;

uint8_t motor_direction_right;
uint8_t motor_direction_left;
int32_t motor_speed_right_qpps;
int32_t motor_speed_left_qpps;
int8_t motor_speed_right;
int8_t motor_speed_left;

int     mixX;              // Joystick X input                     (0..+127)
int     mixY;              // Joystick Y input                     (0..+127)

int adjustedX;
int adjustedY;

int adjustedMotMixR;
int adjustedMotMixL;

// Received Data
int16_t mode_control;     // motor control mode, 0 = open loop, 1 = closed loop, 2 = turret control
int16_t mode_assist;      // autopilot, 0 = off, 1 = adjust, 2 = wander, 3 = roam
int16_t mode_emergency;   // 0 = OK, 1 = full_stop, 2 = full_stop + reset_pid
int16_t mode_lights;      // 0 = OFF, 1 = ambient (white front, red back), 2 = advanced (default + directional blink), 3 = full ON (white everywhere)
int16_t brightness;       // lights intensity, in percentage 0 - 100 (%)
int16_t acceleration;     // maximum PID loop acceleration configuration, in percentage 0 - 100 (%)

// RoboClaw address
#define address 0x80
#define qppsR 9500
#define qppsL 9500
#define RKp 0.2
#define RKi 0.10
#define RKd 0.01
#define LKp 0.2
#define LKi 0.10
#define LKd 0.01

#define MAX_SPEED_R 9500 // in QPPS
#define MAX_SPEED_L 9500 // in QPPS

#define DISTANCE_IN_1S_R 9361 // in QPPS
#define DISTANCE_IN_1S_L 8964 // in QPPS

// Functions
void setupMotorBoard() {
    roboclaw.ResetEncoders(address);
    roboclaw.SetM1VelocityPID(address, RKp, RKi, RKd, qppsR);
    roboclaw.SetM2VelocityPID(address, LKp, LKi, LKd, qppsL);
}

// Program
void setup() {
    lcd.begin(20, 4);

    lcd.setCursor(19, 0);
    lcd.print("V");

    lcd.setCursor(3, 1);
    lcd.print("AI/");

    lcd.setCursor(18, 1);
    lcd.print((char)223); // degree symbol
    lcd.setCursor(19, 1);
    lcd.print("C");

    lcd.setCursor(0, 2);
    lcd.print("R");
    lcd.setCursor(19, 2);
    lcd.print("A");

    lcd.setCursor(0, 3);
    lcd.print("L");
    lcd.setCursor(19, 3);
    lcd.print("A");

    lcd.setCursor(0, 0);
    lcd.print("Rowee is OK");

    roboclaw.begin(38400);
    //Serial1.begin(38400); // To RoboClaw
    //while (!Serial1) { ; }

    servoPan.attach(PIN_SERVO_PAN);
    servoTilt.attach(PIN_SERVO_TILT);
    servoPan.write(SERVO_PAN_CENTER);
    servoTilt.write(SERVO_TILT_CENTER);

    Serial2.begin(9600); // To HC-05 (Bluetooth)
    while (!Serial2) { ; }
    ETin.begin(details(dataRemoteToRobot), &Serial2);
    ETout.begin(details(dataRobotToRemote), &Serial2);

    Serial.begin(9600);
    //while (!Serial) { ; }
    Serial.println("Rowee is ready");

	// p.Begin();
    // p.AddTimeGraph("Rowee", 1500, "speed", speed);
    // p.AddTimeGraph("Rowee", 1500, "x", x);
    // p.AddTimeGraph("Rowee", 1500, "y", y);

    speed = 0;

    maxSpeedR = MAX_SPEED_R;
    maxSpeedL = MAX_SPEED_L;

    setupMotorBoard();

    mode_control = static_cast<int16_t>(ModeControl::OPEN_LOOP);
    mode_assist = static_cast<int16_t>(ModeAssist::OFF);
    mode_emergency = static_cast<int16_t>(ModeEmergency::OK);
    mode_lights = static_cast<int16_t>(ModeLights::OFF);
}

void loop() {
    unsigned long currentMillis = millis();

    // Load data from Remote
    if (ETin.receiveData()) {
        mode_control = dataRemoteToRobot.mode_control;
        mode_assist = dataRemoteToRobot.mode_assist;
        mode_emergency = dataRemoteToRobot.mode_emergency;
        mode_lights = dataRemoteToRobot.mode_lights;
        brightness = dataRemoteToRobot.brightness;
        speed = dataRemoteToRobot.speed;
        x = dataRemoteToRobot.x - 20;
        y = dataRemoteToRobot.y - 5;
        acceleration = dataRemoteToRobot.acceleration;
    }

    // Compute
    limitedSpeed = 950 - min(dataRemoteToRobot.speed, 950);
    normSpeed = map(limitedSpeed, 1, 950, 1, 127);
    percentSpeed = map(normSpeed, 1, 127, 30, 100);
    speedInPercent = percentSpeed / 100.0;

    motor_speed_right_qpps = roboclaw.ReadSpeedM1(address, &motor_direction_right);
    motor_speed_right = abs(round((motor_speed_right_qpps / (MAX_SPEED_R * 1.0)) * 100));
    motor_speed_left_qpps = roboclaw.ReadSpeedM2(address, &motor_direction_left);
    motor_speed_left = abs(round((motor_speed_left_qpps / (MAX_SPEED_L * 1.0)) * 100));

    //Serial.println(speedInPercent);

    //Serial.println(x);
    //Serial.println(y);

    // int distance = sensor.getDistance();

    if (currentMillis - lcdMillis >= INTERVAL_LCD) {
        lcdMillis = currentMillis;

        voltage = roboclaw.ReadMainBatteryVoltage(address);
        lcd.setCursor(15, 0);
        lcd.print(voltage / 10.0, 1);
        //Serial.println(voltage / 10.0);

        roboclaw.ReadCurrents(address, currentR, currentL);
        lcd.setCursor(15, 2);
        lcd.print(currentR / 100.0);
        lcd.setCursor(15, 3);
        lcd.print(currentL / 100.0);
        //Serial.println(currentR / 100.0);
        //Serial.println(currentL / 100.0);

        lcd.setCursor(0, 1);
        if (mode_control == static_cast<int16_t>(ModeControl::OPEN_LOOP)) {
            lcd.print("OL");
        } else if (mode_control == static_cast<int16_t>(ModeControl::CLOSED_LOOP)) {
            lcd.print("CL");
        } else { // TURRET
            lcd.print("TU");
        }

        lcd.setCursor(6, 1);
        if (mode_assist == static_cast<int16_t>(ModeAssist::OFF)) {
            lcd.print("OFF");
        } else if (mode_assist == static_cast<int16_t>(ModeAssist::ADJUST)) {
            lcd.print("ADJ");
        } else if (mode_assist == static_cast<int16_t>(ModeAssist::WANDER)) {
            lcd.print("WAN");
        } else { // ROAM
            lcd.print("ROA");
        }

        roboclaw.ReadTemp(address, temp);
        lcd.setCursor(13, 1);
        lcd.print(temp / 10.0);
        //Serial.println(temp / 100.0);

        if (motor_speed_right == 0) {
            lcd.setCursor(1, 2);
            lcd.print(" ");
        } else {
            if (motor_direction_right == MOTOR_DIRECTION_FORWARD) {
                lcd.setCursor(1, 2);
                lcd.print(">");
            } else {
                lcd.setCursor(1, 2);
                lcd.print("<");
            }
        }

        lcd.setCursor(2, 2);
        lcd.print("    ");
        lcd.setCursor(2, 2);
        lcd.print(motor_speed_right);
        lcd.print("%");

        if (motor_speed_left == 0) {
            lcd.setCursor(1, 3);
            lcd.print(" ");
        } else {
            if (motor_direction_left == MOTOR_DIRECTION_FORWARD) {
                lcd.setCursor(1, 3);
                lcd.print(">");
            } else {
                lcd.setCursor(1, 3);
                lcd.print("<");
            }
        }

        lcd.setCursor(2, 3);
        lcd.print("    ");
        lcd.setCursor(2, 3);
        lcd.print(motor_speed_left);
        lcd.print("%");
    }

    // M1 = Right
    // M2 = Left

    // QPPS
    // M1 - Right = 9361
    // M2 - Left = 8964

    if (mode_emergency == static_cast<int16_t>(ModeEmergency::OK)) {
        lcd.setCursor(0, 0);
        lcd.print("Rowee is OK   ");

        if (mode_control == static_cast<int16_t>(ModeControl::OPEN_LOOP)) {
            // RoboClaw Differential Steering (not perfect) - Open Loop Control
            // nJoyX = map(x, -532, 491, -128, 127);
            // nJoyY = map(y, -517, 506, -128, 127);
            //
            // mixX = map(nJoyX, -128, 127, 0, 127);
            // mixY = map(nJoyY, -128, 127, 0, 127);
            //
            // adjustedX = mixX * speedInPercent;
            // adjustedY = mixY * speedInPercent;
            //
            // if (adjustedY > 5) {
            //     roboclaw.ForwardBackwardMixed(address, adjustedY);
            // } else {
            //     roboclaw.ForwardBackwardMixed(address, 0);
            // }
            //
            // if (adjustedX > 5) {
            //     roboclaw.LeftRightMixed(address, adjustedX);
            // } else {
            //     roboclaw.LeftRightMixed(address, 0);
            // }

            // Inza's Differential Steering - Open Loop Control
            nJoyX = map(x, -532, 491, -128, 127);
            nJoyY = map(y, -517, 506, -128, 127);
            computeDifferentialSteering();

            absMotorR = abs(nMotMixR) * speedInPercent;
            absMotorL = abs(nMotMixL) * speedInPercent;

            if (nMotMixR < -5) {
                roboclaw.BackwardM1(address, absMotorR);
            } else if (nMotMixR > 5) {
                roboclaw.ForwardM1(address, absMotorR);
            } else {
                roboclaw.ForwardM1(address, 0);
            }

            if (nMotMixL < -5) {
                roboclaw.BackwardM2(address, absMotorL);
            } else if (nMotMixL > 5) {
                roboclaw.ForwardM2(address, absMotorL);
            } else {
                roboclaw.ForwardM2(address, 0);
            }
        } else if (mode_control == static_cast<int16_t>(ModeControl::CLOSED_LOOP)) {
            // Inza's Differential Steering - Closed Loop Control
            nJoyX = map(x, -532, 491, -128, 127);
            nJoyY = map(y, -517, 506, -128, 127);
            computeDifferentialSteering();

            // adjustedMotMixR = nMotMixR * speedInPercent;
            // adjustedMotMixL = nMotMixL * speedInPercent;

            if ((nMotMixR < -5) || (nMotMixR > 5) || (nMotMixL < -5) || (nMotMixL > 5)) {
                //Serial.println(nMotMixR);
                //Serial.println(nMotMixL);

                //Serial.println(speedInPercent);

                //roboclaw.DutyM1M2(address, -32767, -32767);

                speedInPercentR = map(nMotMixR, -127, 128, -100, 100) / 100.0;
                speedInPercentL = map(nMotMixL, -127, 128, -100, 100) / 100.0;

                scaledSpeedR = (int32_t)(speedInPercentR * speedInPercent * maxSpeedR);
                scaledSpeedL = (int32_t)(speedInPercentL * speedInPercent * maxSpeedL);

                roboclaw.SpeedDistanceM1M2(address, scaledSpeedR, 100, scaledSpeedL, 100, 1);
            }
        } else { // TURRET
            // TODO for now, robot will stop when in turret mode, in future AI can control its movement
            roboclaw.DutyM1M2(address, 0, 0); // Full Stop

            // TODO control servos
        }
    } else if (mode_emergency == static_cast<int16_t>(ModeEmergency::PID_RESET)) {
        roboclaw.DutyM1M2(address, 0, 0); // Full Stop
        setupMotorBoard();
        lcd.setCursor(0, 0);
        lcd.print("Reset Motors  ");
    } else { // FULL_STOP
        roboclaw.DutyM1M2(address, 0, 0); // Full Stop
        lcd.setCursor(0, 0);
        lcd.print("EMERGENCY STOP");
    }

    // Inza's Simple Duty Control (for testing purposes)
    // nJoyX = map(x, -532, 491, -128, 127);
    // nJoyY = map(y, -517, 506, -128, 127);
    // computeDifferentialSteering();

    // if ((nMotMixR < -5) || (nMotMixR > 5) || (nMotMixL < -5) || (nMotMixL > 5)) {
    //     uint16_t duty = -32767; // Full Forward
    //     roboclaw.DutyM1M2(address, duty, duty);
    // } else {
    //     roboclaw.DutyM1M2(address, 0, 0); // Full Stop
    // }

    if (currentMillis - sendMillis >= INTERVAL_SEND) {
        sendMillis = currentMillis;

        // Send data to Remote
        dataRobotToRemote.max_speed = percentSpeed;     // maximum PID loop speed, in percentage 0 - 100 (%) (computed based on the speed control)
        dataRobotToRemote.max_acceleration = 100;       // maximum PID loop acceleration, in percentage 0 - 100 (%)
        dataRobotToRemote.action = static_cast<int16_t>(AiAction::OFF);
        dataRobotToRemote.battery = 100;
        dataRobotToRemote.motor_right = motor_speed_right;
        dataRobotToRemote.motor_left = motor_speed_left;
        dataRobotToRemote.distance_fr = 0;      // distance Front-Right, in cm
        dataRobotToRemote.distance_fl = 0;      // distance Front-Left, in cm
        dataRobotToRemote.distance_br = 0;      // distance Back-Right, in cm
        dataRobotToRemote.distance_br = 0;      // distance Back-Left, in cm

        ETout.sendData();
    }

    //p.Plot();
}
