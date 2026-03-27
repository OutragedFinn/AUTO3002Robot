#include <Arduino.h>
#include <stdlib.h>
#include <Servo.h>

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

#define MOT_L_EN 2
#define MOT_R_EN 3

#define COL_SERVO1 6
#define COL_SERVO2 7

#define ROT_EN 8
#define LIFT_EN 9
#define ACT_EN1 10
#define ACT_EN2 11

#define MOT_L_FWD 22
#define MOT_L_REV 23
#define MOT_R_FWD 24
#define MOT_R_REV 25

#define LIFT_UP 30
#define LIFT_DOWN 31

#define ROT_IN1 32
#define ROT_IN2 33

#define MOT_L_ENCA 21
#define MOT_R_ENCA 35
#define MOT_L_ENCB 36
#define MOT_R_ENCB 37

#define ACT1_IN1 38
#define ACT1_IN2 39
#define ACT2_IN1 40
#define ACT2_IN2 41

#define SERVO_1 1000;
#define SERVO_2 1001;

// #define ROT_IN1 48
// #define ROT_IN2 49

#define ACT1_ENCA 50
#define ACT1_ENCB 51
#define LIFT_ENC 52
#define ROT_ENC 53

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
State sequence[] = {START, COLLECT, MOVE, LIFT, DROP, RETRACT, RETURN, END};
volatile int stateIndex = 0;

enum class CollectState {
    ALIGN,
    ACTUATOR_EXTEND,
    GRAB,
    ACTUATOR_RETRACT,
    COLLECT_DONE
};

CollectState collectSequence[] = {CollectState::ALIGN, CollectState::ACTUATOR_EXTEND, CollectState::GRAB, CollectState::ACTUATOR_RETRACT, CollectState::COLLECT_DONE};
volatile int collectIndex = 0;

enum class MoveState {
    MOVE_FORWARD,
    ALIGN_WALL
};

enum class DropState {
    ALIGN,
    ROTATE,
};



State currentState;
CollectState currentCollect;


void set_drive(int L_power, int R_power) {
    motor_L_power = L_power;
    motor_R_power = R_power;
}

//fl, fr, bl, br
int mot_ticks[2] = {0,0};
int act_ticks[2] = {0,0};

// int read_colour_sensors() {
//     return 0;
// }

// int check_colour_sensors() {
//     int state = read_colour_sensors();
//     return 1;
// }

volatile bool last_L_A = 0;
void handle_l_mot_ticks() {
    bool A = digitalRead(MOT_L_ENCA);
    bool B = digitalRead(MOT_L_ENCB);

    if (A != last_L_A) {          
        if (A == B) mot_ticks[0]++; 
        else mot_ticks[0]--;
    }
    last_L_A = A;
}

volatile bool last_R_A = 0;
void handle_r_mot_ticks() {
    bool A = digitalRead(MOT_R_ENCA);
    bool B = digitalRead(MOT_R_ENCB);

    if (A != last_R_A) {          
        if (A == B) mot_ticks[0]++; 
        else mot_ticks[0]--;
    }
    last_R_A = A;
}

volatile bool last_act1_A = 0;
void handle_act1_ticks() {
    bool A = digitalRead(ACT1_ENCA);
    bool B = digitalRead(ACT1_ENCB);

    if (A != last_act1_A) {          
    if (A == B) act_ticks[0]++; 
        else act_ticks[0]--;
    }
    last_act1_A = A;

}

volatile int enc_l_old;
volatile int enc_l_new;
volatile int enc_r_old;
volatile int enc_r_new;

volatile int delta_enc_l;
volatile int delta_enc_r;

volatile float error_l = 0;
volatile float error_r = 0;

volatile float prev_error_l = 0;
volatile float prev_error_r = 0;

volatile float integral_l = 0;
volatile float integral_r = 0;

volatile float derivative_l = 0;
volatile float derivative_r = 0;

volatile float v_l_actual;
volatile float v_r_actual;

volatile float v_l_des = 600;
volatile float v_r_des = 600;

volatile float output_l;
volatile float output_r;
float dt = 0.02;

float Kp = 0.5;
float Kd = 0;
float Ki = 0.1;
float I_MAX = 2550;

ISR(TIMER1_COMPA_vect){

    enc_l_new = mot_ticks[0];
    delta_enc_l = enc_l_new - enc_l_old;
    v_l_actual = delta_enc_l / dt;
    enc_l_old = enc_l_new;

    error_l = v_l_des - v_l_actual;

    integral_l += error_l*dt;
    derivative_l = (error_l - prev_error_l)/dt;

    if (integral_l > I_MAX) integral_l = I_MAX;
    if (integral_l < -I_MAX) integral_l = -I_MAX;
    output_l = Kp* error_l + Ki * integral_l + Kd * derivative_l;
    
    prev_error_l = error_l;

    if (output_l > 255) output_l = 255;
    if (output_l < -255) output_l = -255;

    enc_r_new = mot_ticks[1];
    delta_enc_r = enc_r_new - enc_r_old;
    v_r_actual = delta_enc_r / dt;
    enc_r_old = enc_r_new;

    error_r = v_r_des - v_r_actual;

    integral_r += error_r*dt;
    derivative_r = (error_r - prev_error_r)/dt;

    if (integral_r > I_MAX) integral_r = I_MAX;
    if (integral_r < -I_MAX) integral_r = -I_MAX;
    output_r = Kp* error_r + Ki * integral_r + Kd * derivative_r;
    
    prev_error_r = error_r;

    if (output_r > 255) output_r = 255;
    if (output_r < -255) output_r = -255;
    

    // enc_fr_new = mot_ticks[0];
    // delta_enc_fr = enc_fr_new - enc_fr_old;
    // //ticks per second
    // int v_fr_actual = delta_enc_fr*10;

    // enc_fl_old = enc_fl_new;

    // speed_error = v_fl_actual - v_fr_actual;
    // integral+=speed_error;
    // derivative = speed_error - prev_speed_error;
    
    // correction = constrain(Kp*speed_error + Ki*integral + Kd*derivative,-30,30);
    // left_speed  = base_speed - correction;
    // right_speed = base_speed + correction;
    // set_drive(left_speed, right_speed);


};

//drive may be dependant on sensors
void drive_motors() {

    if (currentState == MOVE) {
        analogWrite(MOT_L_EN, int(output_l));
        analogWrite(MOT_R_EN, int(output_r));

        digitalWrite(MOT_L_FWD, HIGH);
        digitalWrite(MOT_L_REV, LOW);

        digitalWrite(MOT_R_FWD, HIGH);
        digitalWrite(MOT_R_REV, LOW);
    }

    if (currentState == RETURN) {
        analogWrite(MOT_L_EN, int(output_l));
        analogWrite(MOT_R_EN, int(output_r));

        digitalWrite(MOT_L_FWD, LOW);
        digitalWrite(MOT_L_REV, HIGH);

        digitalWrite(MOT_R_FWD, LOW);
        digitalWrite(MOT_R_REV, HIGH);
    }
}


volatile int actuatorTicks;


void drive_actuators() {
    if (currentCollect == CollectState::ACTUATOR_EXTEND) {
        if (actuatorTicks >= 100) {
            nextCollect();
        } else {
            analogWrite(ACT_EN1, 255);
            digitalWrite(ACT1_IN1,HIGH);
            digitalWrite(ACT1_IN2,LOW);
        } 
    }
    if (currentCollect == CollectState::ACTUATOR_RETRACT) {
        if (actuatorTicks <= 0) {
            nextCollect();
        } else {
            analogWrite(ACT_EN1, 255);
            digitalWrite(ACT1_IN1,LOW);
            digitalWrite(ACT1_IN2,HIGH);
        } 
    }
}

Servo collectorServo1;
Servo collectorServo2;

void drive_servo() {
    collectorServo1.write(180);
    collectorServo2.write(180);
}

void nextState() {
    stateIndex++;
    currentState = sequence[stateIndex];

}

void nextCollect() {
    collectIndex++;
    currentCollect = collectSequence[collectIndex];
}

void setup() {
    // currentState = START;
    Serial.begin(9600); 

    // Motor control
    pinMode(MOT_L_EN, OUTPUT);
    pinMode(MOT_R_EN, OUTPUT);

    pinMode(MOT_L_FWD, OUTPUT);
    pinMode(MOT_L_REV, OUTPUT);
    // pinMode(MOT_R_FWD, OUTPUT);
    // pinMode(MOT_R_REV, OUTPUT);

    pinMode(MOT_L_ENCA, INPUT_PULLUP);  
    pinMode(MOT_L_ENCB, INPUT_PULLUP);
    
    // pinMode(MOT_R_ENCA, INPUT_PULLUP);  
    // pinMode(MOT_R_ENCB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(MOT_L_ENCA), handle_l_mot_ticks, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOT_R_ENCA), handle_r_mot_ticks, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ACT1_ENCA), handle_act1_ticks, CHANGE);

    TCCR1B = (1<<CS10) | (1<<CS12) | (1<<WGM12);  
    TCNT1 = 0;
    OCR1A = (16000000UL / (1024UL * 50UL)) - 1;
    TIMSK1 |= (1<<OCIE1A);

    collectorServo1.attach(SERVO_1);
    collectorServo2.attach(SERVO_2);

};

void loop() {
    // analogWrite(MOT_L_EN,int(output_l));
    // digitalWrite(MOT_L_REV,0);  
    // Serial.println(v_l_actual);
    // Serial.println(int(output_l));

    // Serial.println(mot_ticks[0]);
    switch (currentState)
    {
    case START:
        nextState();

    case COLLECT:
        currentCollect = CollectState::ALIGN;
        switch (currentCollect)
        {
            case CollectState::ALIGN:
                break;
            case CollectState::ACTUATOR_EXTEND:
                drive_actuators();
                break;
            case CollectState::GRAB:
                drive_servo();
                break;
            case CollectState::ACTUATOR_RETRACT:
                drive_actuators();
                break;
            case CollectState::COLLECT_DONE:
                nextState();
        }
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