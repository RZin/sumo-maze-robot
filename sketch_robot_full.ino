// #4 Task

#include <Encoder.h>

// motor_left
#define BIN1 7
#define BIN2 8
#define PWMB 9

// pulses counting vars
Encoder motorLeft(18, 19);
long last_pulses_num_Left = 0;
long curr_pulses_num_Left;
int pulses_num_left;
int NL;

// motor_right
#define AIN1 5
#define AIN2 6
#define PWMA 4

// pulses counting vars
Encoder motorRight(3, 2);
long last_pulses_num_Right = 0;
long curr_pulses_num_Right;
int pulses_num_right;
int NR;

// total robots distances
unsigned long total_pulses_num_Left = 0;
unsigned long total_pulses_num_Right = 0;

// timing
unsigned long prev_count_Millis = 0;
unsigned long curr_Millis;

//sensors
const int LED_pin = 13;
const int IR_L_pin = A2;
const int IR_F_pin = A3;
const int IR_R_pin = A4;
const int line_F_pin = A5;
const int line_B_pin = A6;

// task constants
const float counting_freq = 10.0;
const float left_pulses_correction = 1.00; // correction coeficient

void setup() {
    // put your setup code here, to run once:
    pinMode(LED_pin, OUTPUT);
    pinMode(IR_L_pin, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    Serial.begin(9600);
}

int IR_to_mm(float IR_raw){
    // 320, 1.1 and 3.7 params found experimentally using curve fitting
    // returns distance in mm
    return (pow(320/IR_raw, 1.1) + 3.7)*10;
}

// 1.
int sense_L () {
    return IR_to_mm(map(analogRead(IR_L_pin), 0, 1023, 0, 100));
}
// 2.
int sense_F () {
    return IR_to_mm(map(analogRead(IR_F_pin), 0, 1023, 0, 100));
}
// 2.
int sense_R () {
    return IR_to_mm(map(analogRead(IR_R_pin), 0, 1023, 0, 100));
}
// 3.
boolean sense_line_F () {
    return (analogRead(line_F_pin)>500);
}
// 4.
boolean sense_line_B () {
    return (analogRead(line_B_pin)>500);
}
// 5.
void count_pulses(){

    // call every 100ms for 10Hz
    curr_pulses_num_Left = motorLeft.read();
    curr_pulses_num_Right = motorRight.read();

    // count
    pulses_num_left = curr_pulses_num_Left - last_pulses_num_Left;
    pulses_num_right = curr_pulses_num_Right - last_pulses_num_Right;
    NL = pulses_num_left*left_pulses_correction;
    NR = pulses_num_right;

    // update vars
    last_pulses_num_Left = curr_pulses_num_Left;
    last_pulses_num_Right = curr_pulses_num_Right;

    total_pulses_num_Left += abs(pulses_num_left);
    total_pulses_num_Right += abs(pulses_num_right);
}

void test_sensors(){
    // testing sensors data
    Serial.print(" dist_L ");
    Serial.print(sense_L ());
    Serial.print(" dist_F ");
    Serial.print(sense_F ());
    Serial.print(" dist_R ");
    Serial.print(sense_R ());
    Serial.print(" line_F_val ");
    Serial.print(sense_line_F ());
    Serial.print(" line_B_val ");
    Serial.print(sense_line_B ());
    Serial.println();
    delay(5000);
}

//
// C O N T R O L
//

// time incrementor
int t = 0;

// pulses per revolution
float CL=8000;
float CR=8000;
float C=(CL+CR)/2;

// positions
float x;
float y;
float theta;

//displacement
float D;

// robot dimensions
const float b = 0.095; //cm 0.095 mm 20
const float r = 0.015; // cm 0.015 mm 5

float VD = 0.0; // 0.08 max m/s
float WD = 0.3; // 2 max rad/s

//float VMAX = 0.08; // m/s
//float WMAX = 3.14/2; // rad/s PI*r half of round 1,57

//define vels
float v;
float w;

float wL;
float wR;

//define des vels
float vd;
float wd;

float wLd;
float wRd;

const float delta_t = 1/counting_freq; // in seconds

void cmd_vel(){
    // calculate current velocities from pulses on encoders
    v = (2*PI*r)*(NR+NL)/C/2/delta_t;
    w = (2*PI*r)*(NR-NL)/C/b/delta_t;
}

void vel_to_wheels(float V, float W){
    // convert current velocities to wheels' rot vels
    wL = (float)(V-b*W/2)/r;
    wR = (float)(V+b*W/2)/r;
}

void vel_to_wheels_desired(float V_des, float W_des){
    // convert current desired velocities to desired wheels' rot vels
    wLd = (float)(V_des-b*W_des/2)/r;
    wRd = (float)(V_des+b*W_des/2)/r;
}

void posUpdate(int NL, int NR, float r, float b, float C) {
    // update position using pulses on encoders

    // define displacements of the Wheels
    float DL = 2*PI*r*NL/CL;
    float DR = 2*PI*r*NR/CR;
    float D = (DL+DR)/2;

    // update position
    x = x + D * cos(theta) ;
    y = y + D * sin(theta) ;
    theta = theta + (DR-DL)/b ;
}

void calc_pose2(){
    // get encoder pulses NL NR
    count_pulses();

    // update linear velocity and angular velocity
    v = (2*PI*r)*(NR+NL)/C/2/delta_t; // (2*3.14*0.015)*40000/8000/2/0.1
    w = (2*PI*r)*(NR-NL)/C/b/delta_t; //

    // update position and orientation
    theta = atan2(sin(theta + w), cos(theta + w));
    x = x + v * cos(theta); // should be *delta_t in seconds
    y = y + v * sin(theta);
}


// gain constants
float G[2]={0, 0};
// define errors
float ep_curr[2] = {0, 0};
float ep_prev[2] = {0, 0};
float ei[2] = {0, 0};
float ed[2] = {0, 0};

void PID (){
    // coefs Kp
    float Kp = 70.0;
    float Ki = 20.0;
    float Kd = 10.0;

    // update errors
    // We have wL wR

    // save prev
    ep_prev[0] = ep_curr[0];
    ep_prev[1] = ep_curr[1];
    // left proportional
    ep_curr[0] = wLd - wL; // in hz
    // right proportional
    ep_curr[1] = wRd - wR;
    // left integral
    ei[0] = (ei[0] + ep_curr[0])*delta_t;
    // right integral
    ei[1] = (ei[1] + ep_curr[1])*delta_t;
    // left derivative
    ed[0] = (ep_curr[0] - ep_prev[0])/delta_t;
    // right derivative
    ed[1] = (ep_curr[1] - ep_prev[1])/delta_t;

    // update gain based on errors
    G[0] = G[0] + Kp*ep_curr[0] + Ki*ei[0] + Kd*ed[0]; // hz
    G[1] = G[1] + Kp*ep_curr[1] + Ki*ei[1] + Kd*ed[1];
}

void encFunc() {t++;}

void pid_controller(){

    // get wLd wRd
    vel_to_wheels_desired(VD, WD);

    // get v w theta x y
    calc_pose2();

    // calculate the real wL, wR
    vel_to_wheels(v, w);

    // run PID and get G[2]
    PID();
}

// movement
void rotate_by_combination(int comb[4], int m_left_speed, int m_right_speed) {
    // comb[4] takes states of {AIN1 AIN2 BIN1 BIN2} (private in future)
    if (0 <= m_left_speed <= 255 && 0 <= m_right_speed <= 255) {
        analogWrite(PWMB, m_left_speed); // m_left  speed
        analogWrite(PWMA, m_right_speed); // m_right speed
    }
    else { Serial.println("Speed is Out of limits !!!");}
    digitalWrite(BIN1, comb[0]); // m_left clockwise
    digitalWrite(BIN2, comb[1]); // m_left anticlockwise
    digitalWrite(AIN1, comb[2]); // m_right  clockwise
    digitalWrite(AIN2, comb[3]); // m_right  anticlockwise
}

void move_at(float speed_left, float speed_right) {

    int comb[4]={0};

    // cut exceeding vals
    if (speed_left > 255) {
        speed_left = 255;
    }
    if (speed_left < -255) {
        speed_left = -255;
    }
    if (speed_right > 255) {
        speed_right = 255;
    }
    if (speed_right < -255) {
        speed_right = -255;
    }

    // ser combination
    if (speed_left>0 && speed_right>0){
        comb[0] = 1;
        comb[1] = 0;
        comb[2] = 1;
        comb[3] = 0;
    }
    else if (speed_left>0 && speed_right<0){
        comb[0] = 1;
        comb[1] = 0;
        comb[2] = 0;
        comb[3] = 1;
    }
    else if (speed_left<0 && speed_right>0){
        comb[0] = 0;
        comb[1] = 1;
        comb[2] = 1;
        comb[3] = 0;
    }
    else if (speed_left<=0 && speed_right<=0){
        comb[0] = 0;
        comb[1] = 1;
        comb[2] = 0;
        comb[3] = 1;
    }

    else {
        Serial.print(" Unexpected combination of speeds ");
        Serial.print(speed_left);
        Serial.print(speed_right);
        Serial.println();
    }

    rotate_by_combination(comb, abs(speed_left), abs(speed_right));
}

// movements at maximum speed
void moveForward() {
    int comb[4] = {1, 0, 1, 0};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({1, 0, 1, 0});
}

void moveBackward() {
    int comb[4] = {0, 1, 0, 1};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({0, 1, 0, 1});
}

void turnRight() {
    int comb[4] = {1, 0, 0, 1};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({1, 0, 1, 0});
}

void turnLeft() {
    int comb[4] = {0, 1, 1, 0};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({1, 0, 1, 0});
}

void stopMoving() {
    int comb[4] = {0, 0, 0, 0};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({1, 0, 1, 0});
}

void stopFast() {
    int comb[4] = {1, 1, 1, 1};
    rotate_by_combination (comb, 255, 255);
    // change to just rotate_by_combination ({1, 0, 1, 0});
}

void moveForwardRight () {
    int comb[4] = {1, 0, 0, 0};
    rotate_by_combination(comb, 255, 255);
}

void moveBackwardRight () {
    int comb[4] = {0, 1, 0, 0};
    rotate_by_combination(comb, 255, 255);
}

void moveForwardLeft () {
    int comb[4] = {0, 0, 1, 0};
    rotate_by_combination(comb, 255, 255);
}

void moveBackwardLeft () {
    int comb[4] = {0, 0, 0, 1};
    rotate_by_combination(comb, 255, 255);
}

void test_movements(){
    // testing moving functions
    turnRight();
    delay(2150);
    stopFast();
    count_pulses(); // with given freq

    Serial.print("pulses_num_left: ");
    Serial.print(pulses_num_left);
    Serial.println();
}


void loop() {

    curr_Millis=millis();

    if (curr_Millis-prev_count_Millis >= 1000/counting_freq) {

        pid_controller();

        move_at(G[0], G[1]);

        prev_count_Millis = curr_Millis;

    }

    Serial.print(w*100);
//    Serial.print(v*100);
    Serial.println();
}

