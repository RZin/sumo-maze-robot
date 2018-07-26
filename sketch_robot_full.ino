// #3 Task

#include <Encoder.h>;

// motor_left
#define BIN1 7 
#define BIN2 8 
#define PWMB 9

// pulses counting vars
Encoder motorLeft(18, 19);
long last_pulses_num_Left  = 0;
long curr_pulses_num_Left;
int pulses_num_left;
int NL = pulses_num_right; // not sure of pointer

// motor_right
#define AIN1 5 
#define AIN2 6  
#define PWMA 4 

// pulses counting vars
Encoder motorRight(3, 2);
long last_pulses_num_Right = 0;
long curr_pulses_num_Right;
int pulses_num_right;
int NR = pulses_num_right;

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
const int counting_freq = 10; //Hz set 10

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
  attachInterrupt(digitalPinToInterrupt(2), count_pulses_interrupt, RISING); // mb LOW or RISING
  attachInterrupt(digitalPinToInterrupt(3), count_pulses_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(18), count_pulses_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(19), count_pulses_interrupt, RISING);
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
  
  curr_Millis=millis();
  
  if (curr_Millis-prev_count_Millis >= 1000/counting_freq) {
    
    // call every 100ms for 10Hz 
    curr_pulses_num_Left = motorLeft.read();  
    curr_pulses_num_Right = motorRight.read();
    
    // count 
    pulses_num_left = curr_pulses_num_Left - last_pulses_num_Left;  
    pulses_num_right = curr_pulses_num_Right - last_pulses_num_Right;
    
    // update vars
    last_pulses_num_Left = curr_pulses_num_Left;
    last_pulses_num_Right = curr_pulses_num_Right;
    
    total_pulses_num_Left += abs(pulses_num_left);
    total_pulses_num_Right += abs(pulses_num_right);
    
    // update timer
    prev_count_Millis=curr_Millis;
  }
}

void test_sensors(){
  // testing sensors data
  Serial.print("dist_L ");
  Serial.println(sense_L ());    
  Serial.print("dist_F ");
  Serial.println(sense_F ());  
  Serial.print("dist_R ");
  Serial.println(sense_R ());  
  Serial.print("line_F_val ");
  Serial.println(sense_line_F ());  
  Serial.print("line_B_val ");
  Serial.println(sense_line_B ());
  delay(5000);    
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

void moveForward() {
  int front_left_comb[4] = {1, 0, 1, 0};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({1, 0, 1, 0});
}

void moveBackward() {
  int front_left_comb[4] = {0, 1, 0, 1};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({0, 1, 0, 1});
}

void turnRight() {
  int front_left_comb[4] = {1, 0, 0, 1};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({1, 0, 1, 0});
}

void turnLeft() {
  int front_left_comb[4] = {0, 1, 1, 0};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({1, 0, 1, 0});
}

void stopMoving() {
  int front_left_comb[4] = {0, 0, 0, 0};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({1, 0, 1, 0});
}

void stopFast() {
  int front_left_comb[4] = {1, 1, 1, 1};
  rotate_by_combination (front_left_comb);
  // change to just rotate_by_combination ({1, 0, 1, 0});
}

void moveForwardRight () {
  int front_left_comb[4] = {1, 0, 0, 0};
  rotate_by_combination(front_left_comb);
}

void moveBackwardRight () {
  int front_right_comb[4] = {0, 1, 0, 0};
  rotate_by_combination(front_right_comb);
}

void moveForwardLeft () {
  int front_left_comb[4] = {0, 0, 1, 0};
  rotate_by_combination(front_left_comb);
}

void moveBackwardLeft () {
  int front_right_comb[4] = {0, 0, 0, 1};
  rotate_by_combination(front_right_comb);
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


//
// C O N T R O L
//

float NL=0;
float NR=0;

float CL=8100; // pulses per revolution
float CR=8000;

int t=0;

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

float x[10]={0};
float y[10]={0};
float theta[10]={0};

float D[10]={0};

float v[10]={0};
float w[10]={0};

float VD = 200; // 255 max
float WD = 0.0; //

//float vd[10]={1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5};
//float wd[10]={1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};

//define vels
float wL[10]={0};
float wR[10]={0};

float v_curr= 0;
float w_curr= 0;

float wL_curr= 0;
float wR_curr= 0;

//define des vels
float wLd[10] = {0};
float wRd[10] = {0};

float vd_curr= 0;
float wd_curr= 0;

float wLd;
float wRd;

const float b=20; //cm
const float r=5; // cm
float C=(CL+CR)/2; // not sure but for me 8000 was 100

void encUpdate(int t) {
    NL = N[t][0]; // read
    NR = N[t][1];
}

const int delta_t = 1000/counting_freq;

void cmd_vel(){
    v_curr = (float)(NL+NR)*PI*r/C/delta_t;
    w_curr = (float)(NL-NR)*PI*r/C/delta_t*2/b;
}

void vel_to_wheels(float V, float W){
    wL_curr = (float)(V-b*W/2)/r;
    wR_curr = (float)(V+b*W/2)/r;
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
    x[t+1] = x[t] + D * cos(theta[t]) ;
    y[t+1] = y[t] + D * sin(theta[t]) ;
    theta[t+1] = theta[t] + (DR-DL)/b ;

    // get v_curr, w_curr
    cmd_vel();

    // get wL_curr, wR_curr
    vel_to_wheels(v_curr, w_curr); // wL_curr wR_curr
}


void calc_pose2(){
    // get NL NR
    count_pulses();

    v[t+1] = (2*PI*r)*(NR+NL)/C/2/delta_t;
    w[t+1] = (2*PI*r)*(NR-NL)/C/b/delta_t;

    theta[t+1] = atan2(sin(theta[t]+w[t+1]), cos(theta[t]+w[t+1]));
    x[t+1] = x[t] + v[t+1] * cos(theta[t+1]);
    y[t+1] = y[t] + v[t+1] * sin(theta[t+1]);
}

// gain constants
float GL[10] = {0};
float GR[10] = {0};


void PID (){
    // define errors
    float ep[10][2] =
            {{wLd[0] - wL[0], wRd[0] - wR[0]}};

    float ei[10][2] =
            {{0,0}};

    float ed[10][2] =
            {{0,0}};

    // coefs Kp
    float Kp = 0.5;
    float Ki = 0;
    float Kd = 0;

    // update errors

    // We have wL_curr wR_curr

    // left proportional
    ep[t+1][0] = wLd_curr - wL_curr;
    // right proportional
    ep[t+1][1] = wRd_curr - wR_curr;
    // left integral
    ei[t+1][0] = (ei[t][0] + ep[t+1][0])*delta_t; // delta_t or 1
    // right integral
    ei[t+1][1] = (ei[t][1] + ep[t+1][1])*delta_t;

    // left derivative
    ed[0][t+1] = (ep[t+1][0] - ep[t][0])/delta_t;
    // right derivative
    ed[1][t+1] = (ep[t+1][1] - ep[t][1])/delta_t;

    // get the gain
    GL[t+1] = Kp*ep[t+1][0] + Ki*ei[t+1][0] + Kd*ed[t+1][0];
    GR[t+1] = Kp*ep[t+1][1] + Ki*ei[t+1][1] + Kd*ed[t+1][1];
}


int t = 0;

void encFunc() {t++;}

void pid_controller(){
    // 1. set desired
    VD = 200; // by now constant
    WD = 0;
    // 2. get wLd_curr wRd_curr
    vel_to_wheels_desired(VD, WD); // updates wLd_curr wRd_curr
    // 3. Run algorithm Calculate Pose 2
    calc_pose2(); // get v[t+1] w[t+1] theta[t+1] x[t+1] y[t+1]
    // 4. Calculate the real wL, wR
    cmd_vel(); // updates v_curr, w_curr
    vel_to_wheels(v_curr, w_curr); // updates wL_curr wR_curr
    // run PID
    PID(); // gets GL GR

}

void loop() {

    curr_Millis=millis();

    if (curr_Millis-prev_count_Millis >= 1000/counting_freq) {
        encFunc();
        count_pulses(); // NL NR
        pid_controller();
    }

}

