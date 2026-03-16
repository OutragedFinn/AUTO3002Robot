#include <Arduino.h>
#include <stdlib.h>

#define IR_FL_A A0
#define IR_BL_A A1
#define IR_FR_A A2
#define IR_BR_A A3

#define IR_PIPE A4
#define IR_PLATF A5

#define IR_FL_COL A8
#define IR_BL_COL A9
#define IR_FR_COL A10
#define IR_BR_COL A11

#define MOT_FL_EN 2
#define MOT_BL_EN 3
#define MOT_FR_EN 4
#define MOT_BR_EN 5

#define COL_SERVO1 6
#define COL_SERVO2 7

#define ROT_EN 8
#define LIFT_EN 9
#define ACT_EN1 10
#define ACT_EN2 11

#define MOT_FL_IN1 22
#define MOT_FL_IN2 23
#define MOT_BL_IN1 24
#define MOT_BL_IN2 25
#define MOT_FR_IN1 26
#define MOT_FR_IN2 27
#define MOT_BR_IN1 28
#define MOT_BR_IN2 29

#define LIFT_IN1 30
#define LIFT_IN2 31

#define ROT_IN1 32
#define ROT_IN2 33

#define MOT_FL_ENC 34
#define MOT_BL_ENC 35
#define MOT_FR_ENC 36
#define MOT_BR_ENC 37

#define ACT1_IN1 38
#define ACT1_IN2 39
#define ACT2_IN1 40
#define ACT2_IN2 41

#define ROT_IN1 48
#define ROT_IN2 49

#define ACT1_ENC 50
#define ACT2_ENC 51
#define LIFT_ENC 52
#define ROT_ENC 53

enum State {
    START,
    COLLECT,
    MOVE,
    LIFT,
    DROP,
    RETRACT,
    RETURN,
    END

};

State currentState;

void setup() {
    currentState = START;
}

void loop() {
    switch (currentState)
    {
    case START:
        break;
    case COLLECT:
        break;
    case MOVE:
        break;
    case LIFT:
        break;
    case DROP:
        break;
    case RETRACT:
        break;
    case RETURN:
        break;
    case END:
        break;
    }
}