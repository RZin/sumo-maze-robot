// #3 Task

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
int NL; // not sure of pointer

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

unsigned long total_pulses_num_Left = 0; // total robots distance
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
const float counting_freq = 1.0; //Hz set 10
const float left_pulses_correction = 0.96;

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
//  attachInterrupt(digitalPinToInterrupt(2), count_pulses_interrupt, RISING); // mb LOW or RISING
//  attachInterrupt(digitalPinToInterrupt(3), count_pulses_interrupt, RISING);
//  attachInterrupt(digitalPinToInterrupt(18), count_pulses_interrupt, RISING);
//  attachInterrupt(digitalPinToInterrupt(19), count_pulses_interrupt, RISING);
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
  // return mm
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
  return (analogRead(line_F_pin)>500); // or without brackets
}
// 4.
boolean sense_line_B () {
  return (analogRead(line_B_pin)>500); // or without brackets
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
//    Serial.print(" NL ");
//    Serial.print(NL);
//    Serial.print(" NR ");
//    Serial.print(NR);
//    Serial.println();
    
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


float CL=8000; // pulses per revolution
float CR=8000;

int t = 0;

float N[10][2] =
        {
                {0,0},
                {100,100},
                {300,200},
                {500,300},
                {700,400},
                {900,500},
                {1000,600},
                {1100,700},
                {1200,800},
        };

// positions 
float x_curr;
float x_prev;
float y_curr;
float y_prev;
float theta_curr;
float theta_prev;

float D;

// robot dimensions
const float b = 0.095; //cm 0.095 mm 20
const float r = 0.015; // cm 0.015 mm 5

float C=(CL+CR)/2; // not sure but for me 8000 was 100

double VD = 0.0; // 0.08 max m/s
double WD = 0.7; // 2 max rad/s

float VMAX = 0.08; // m/s
float WMAX = 3.14/2; // rad/s PI*r half of round 1,57

float WLR_MAX = 8.935;    // wheels max vel
//define vels
double v_curr;
double w_curr;
double v_prev=0;
double w_prev=0;

double wL_curr;
double wR_curr;

//define des vels
double vd_curr;
double wd_curr;

double wLd_curr;
double wRd_curr;

void encUpdate(int t) {
    NL = N[t][0]; // read
    NR = N[t][1];
}

const float delta_t = 1/counting_freq; // seconds

void cmd_vel(){
    v_curr = (float)(NL+NR)*PI*r/C/delta_t;
    w_curr = (float)(NL-NR)*PI*r/C/delta_t*2/b;
}

void vel_to_wheels(float V, float W){
    wL_curr = (float)(V-b*W/2)/r;
    wR_curr = (float)(V+b*W/2)/r;
    // 0.08/0.015 + 0.095*1,57/2/0.015 = 5,33 + 4,97 = 10.3 Hz
    // mb wL_currMAX = 5.5
}

void vel_to_wheels_desired(float V_des, float W_des){
    wLd_curr = (float)(V_des-b*W_des/2)/r; 
    wRd_curr = (float)(V_des+b*W_des/2)/r;
}


void posUpdate(int NL, int NR, float r, float b, float C) {

    float DL = 2*PI*r*NL/CL;
    float DR = 2*PI*r*NR/CR;

    float D=(float)(DL+DR)/2;

    // not sure
    x_curr = x_prev + D * cos(theta_prev) ;
    y_curr = y_prev + D * sin(theta_prev) ;
    theta_curr = theta_prev + (DR-DL)/b ;

    // get v_curr, w_curr
    cmd_vel();

    // get wL_curr, wR_curr
    vel_to_wheels(v_curr, w_curr); // wL_curr wR_curr
}


void calc_pose2(){
    // get NL NR
    count_pulses();
//    Serial.print(" NL ");
//    Serial.print(NL);
//    Serial.print(" NR ");
//    Serial.print(NR);
//    Serial.println();

    v_curr = (2*PI*r)*(NR+NL)/C/2/delta_t; // (2*3.14*0.015)*40000/8000/2/0.1
    w_curr = (2*PI*r)*(NR-NL)/C/b/delta_t; // 
//    Serial.print(" v_curr ");
//    Serial.print(v_curr);    
//    Serial.print(" w_curr ");
//    Serial.print(w_curr);    
//    Serial.println();

    theta_curr = atan2(sin(theta_prev + w_curr), cos(theta_prev + w_curr));
    x_curr = x_prev + v_curr * cos(theta_curr);
    y_curr = y_prev + v_curr * sin(theta_curr);
}

// gain constants
float G_curr[2]={0,0};

// define errors at t=1
float ep_curr[2];
float ep_prev[2] = {0, 0};

float ei_curr[2];
float ei_prev[2] = {0, 0};

float ed_curr[2];
float ed_prev[2] = {0, 0};


void PID (){
    // coefs Kp
    float Kp = 30.0;
    float Ki = 1.8;
    float Kd = 1.0;

    // update errors
    // We have wL_curr wR_curr

    // left proportional
    ep_curr[0] = wLd_curr - wL_curr; // hz
    // right proportional
    ep_curr[1] = wRd_curr - wR_curr;
    // left integral
    ei_curr[0] = (ei_prev[0] + ep_curr[0])*delta_t; // delta_t or 1
    // right integral
    ei_curr[1] = (ei_prev[1] + ep_curr[1])*delta_t; // delta_t or 1
    // left derivative
    ed_curr[0] = (ep_curr[0] - ep_prev[0])/delta_t;
    // right derivative
    ed_curr[1] = (ep_curr[1] - ep_prev[1])/delta_t;

    // get the gain
//    G_prev = G_curr; // hz 
        
    G_curr[0] = G_curr[0] + Kp*ep_curr[0] + Ki*ei_curr[0] + Kd*ed_curr[0]; // hz 
    G_curr[1] = G_curr[1] + Kp*ep_curr[1] + Ki*ei_curr[1] + Kd*ed_curr[1];
    


    
}

void encFunc() {t++;}

void pid_controller(){
    // 1. set desired
//    VD = 0.03; // by now constant m/s
//    WD = 0.0;
    // 2. get wLd_curr wRd_curr
    vel_to_wheels_desired(VD, WD); // updates wLd_curr wRd_curr
//    Serial.print(" wLd_curr ");
//    Serial.print(wLd_curr);    
//    Serial.print(" wRd_curr ");
//    Serial.print(wRd_curr);    
//    Serial.println();
    // 3. Run algorithm Calculate Pose 2
    calc_pose2(); // get v w theta x y

    
    // 4. Calculate the real wL, wR
    vel_to_wheels(v_curr, w_curr); // updates wL_curr wR_curr
//    Serial.print(" wL_curr ");
//    Serial.print(wL_curr);    
//    Serial.print(" wR_curr ");
//    Serial.print(wR_curr);    
//    Serial.println();
    // run PID
    PID(); // gets GL GR
//    Serial.print(" GL_curr ");
//    Serial.print(G_curr[0]);
//    Serial.print(" GR_curr ");
//    Serial.print(G_curr[1]);
//    Serial.println();
      
// use limit for G

}

// for movement
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
    // speed_left
    
    int comb[4]={};
    // cut exceeding vals
    if (speed_left >= 255) { // -800 
      speed_left = 255; 
    }
    if (speed_left <= -255) {
      speed_left = -255;
    }
    if (speed_right >= 255) {
      speed_right = 255;
    }
    if (speed_right <= -255) {
      speed_right = -255;
    }

    if (speed_left>0 && speed_right>0){
        comb[0] = 1;
        comb[1] = 0;
        comb[2] = 1;
        comb[3] = 0;
//        Serial.println(" comb[4] = {1, 0, 1, 0}");
    }
    else if (speed_left>0 && speed_right<0){
//        int comb[4] = {1, 0, 0, 1};
        comb[0] = 1;
        comb[1] = 0;
        comb[2] = 0;
        comb[3] = 1;
//        Serial.println(" comb[4] = {1, 0, 0, 1}");
    }
    else if (speed_left<0 && speed_right>0){
//        int comb[4] = {0, 1, 1, 0};
        comb[0] = 0;
        comb[1] = 1;
        comb[2] = 1;
        comb[3] = 0;
//        Serial.println(" comb[4] = {0, 1, 1, 0}");
    }
    else if (speed_left<=0 && speed_right<=0){
//        int comb[4] = {0, 1, 0, 1};
        comb[0] = 0;
        comb[1] = 1;
        comb[2] = 0;
        comb[3] = 1;
//        Serial.println(" comb[4] = {0, 1, 0, 1}");
    }
    else {
      Serial.print(" Unexpected combination of speeds ");
      Serial.print(speed_left);
      Serial.print(speed_right);
      Serial.println();
      }


//    Serial.print(" speed_left ");  
//    Serial.print(speed_left);  
//    Serial.print(" speed_right "); 
//    Serial.print(speed_right); 
//    Serial.println();

    rotate_by_combination(comb, abs(speed_left), abs(speed_right));


}

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

//        count_pulses(); // NL NR
//        Serial.println(NL);
//        Serial.println(NR);

        pid_controller();
//
//        encFunc();
        // update timer
        
        move_at(G_curr[0], G_curr[1]);
        
        prev_count_Millis=curr_Millis;
        
          
//        Serial.print(" v_curr ");
//        Serial.print(v_curr*100);

    }

    Serial.print(w_curr*100);
//    Serial.print(v_curr*100);
    Serial.println();

//    moveForward();
//    delay(5000);
//    delay(2100);
//    turnRight();

//    stopFast();
//    delay(7000);
//    
}

//23/5s pi/4 rad/s














