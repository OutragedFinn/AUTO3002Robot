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

#define MOT_FL_FWD 22
#define MOT_FL_REV 23
// #define MOT_BL_FWD 24
// #define MOT_BL_REV 25
#define MOT_FR_FWD 24
#define MOT_FR_REV 25
// #define MOT_BR_FWD 28
// #define MOT_BR_REV 29

#define LIFT_UP 30
#define LIFT_DOWN 31

#define ROT_IN1 32
#define ROT_IN2 33

#define MOT_FL_ENCA 34
// #define MOT_BL_ENCA 36
#define MOT_FR_ENCA 35
// #define MOT_BR_ENCA 40

#define MOT_FL_ENCB 36
// #define MOT_BL_ENCB 37
#define MOT_FR_ENCB 37
// #define MOT_BR_ENCB 41

// #define ACT1_IN1 38
// #define ACT1_IN2 39
// #define ACT2_IN1 40
// #define ACT2_IN2 41

// #define ROT_IN1 48
// #define ROT_IN2 49

// #define ACT1_ENC 50
// #define ACT2_ENC 51
// #define LIFT_ENC 52
// #define ROT_ENC 53

int motor_L_ticks;
int motor_R_ticks;

int motor_L_power = 0; 
int motor_R_power = 0; 

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

void drive_motors() {

    if

}

void set_drive(int L_power, int R_power) {
    motor_L_power = L_power;
    motor_R_power = R_power;
}

//fl, fr, bl, br
int mot_ticks[2] = {0,0};

int read_colour_sensors() {
    return 0;
}

int check_colour_sensors() {
    int state = read_colour_sensors();
    return 1;
}

void handle_fl_mot_ticks() {
    if (digitalRead(MOT_FL_ENCB)) {
        mot_ticks[0]++;

    } else {
        mot_ticks[0]--;

    }
}

void handle_fr_mot_ticks() {
    if (digitalRead(MOT_FR_ENCB)) {
        mot_ticks[1]++;
    } else {
        mot_ticks[1]--;
    }  
}
// void handle_bl_mot_ticks() {
//     if (digitalRead(MOT_BL_ENCB)) {
//         mot_ticks[2]++;
//     } else {
//         mot_ticks[2]--;
//     }  
// }
// void handle_br_mot_ticks() {
//     if (digitalRead(MOT_BR_ENCB)) {
//         mot_ticks[3]++;
//     } else {
//         mot_ticks[3]--;
//     }  
// }


volatile int enc_fl_old;
volatile int enc_fl_new;
volatile int enc_fr_old;
volatile int enc_fr_new;
volatile int speed_error = 0;
volatile int integral = 0;
volatile int derivative = 0;
volatile int prev_speed_error = 0;
volatile int correction;
volatile int delta_enc_fl;
volatile int delta_enc_fr;
volatile int v_fl_actual;
volatile int v_fr_actual;
volatile int left_speed;
volatile int right_speed;
//percentage
int base_speed = 70;

int Kp = 3;
int Kd = 3;
int Ki = 3;

ISR(TIMER1_COMPA_vect){

    enc_fl_new = mot_ticks[0];
    delta_enc_fl = enc_fl_new - enc_fl_old;
    //ticks per second
    int v_fl_actual = delta_enc_fl*10;

    enc_fl_old = enc_fl_new;

    enc_fr_new = mot_ticks[0];
    delta_enc_fr = enc_fr_new - enc_fr_old;
    //ticks per second
    int v_fr_actual = delta_enc_fr*10;

    enc_fl_old = enc_fl_new;

    speed_error = v_fl_actual - v_fr_actual;
    integral+=speed_error;
    derivative = speed_error - prev_speed_error;
    
    correction = constrain(Kp*speed_error + Ki*integral + Kd*derivative,-30,30);
    left_speed  = base_speed - correction;
    right_speed = base_speed + correction;
    set_drive(left_speed, right_speed)


}

void setup() {
    currentState = START;

    //Motors
    pinMode(MOT_BL_EN, OUTPUT);
    pinMode(MOT_BR_EN, OUTPUT);
    pinMode(MOT_FL_EN, OUTPUT);
    pinMode(MOT_FL_EN, OUTPUT);

    // pinMode(MOT_BL_FWD, OUTPUT);
    // pinMode(MOT_BL_REV, OUTPUT);
    pinMode(MOT_FL_FWD, OUTPUT);
    pinMode(MOT_FL_REV, OUTPUT);
    // pinMode(MOT_BR_FWD, OUTPUT);
    // pinMode(MOT_BR_REV, OUTPUT);
    pinMode(MOT_FR_FWD, OUTPUT);
    pinMode(MOT_FR_REV, OUTPUT);
    // pinMode(MOT_BR_FWD, OUTPUT);
    // pinMode(MOT_BR_REV, OUTPUT);

    pinMode(MOT_FL_ENCA, INPUT_PULLUP);
    pinMode(MOT_FR_ENCA, INPUT_PULLUP);
    // pinMode(MOT_BL_ENCA, INPUT_PULLUP);
    // pinMode(MOT_BR_ENCA, INPUT_PULLUP);
    
    pinMode(MOT_FL_ENCB, INPUT_PULLUP);
    pinMode(MOT_FR_ENCB, INPUT_PULLUP);
    // pinMode(MOT_BL_ENCB, INPUT_PULLUP);
    // pinMode(MOT_BR_ENCB, INPUT_PULLUP);

    // Setup timer for motor control
    attachInterrupt(digitalPinToInterrupt(MOT_FL_ENCA), handle_fl_mot_ticks, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOT_FR_ENCA), handle_fr_mot_ticks, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(MOT_BL_ENCA), handle_bl_mot_ticks, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(MOT_BR_ENCA), handle_br_mot_ticks, CHANGE);

    TCCR1B = (1<<CS10) | (1<<CS12) | (1<<WGM12); // Prescaler 1024 and timer counter reset on compare value
	TCNT1 = 0; // set initial counter value to 0
	OCR1A = (16000000/(1024*1))-1; // set compare value to 15624
	TIMSK1 |= (1<<OCIE1A);

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