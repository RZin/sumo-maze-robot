// LAST MASTER
#include <Encoder.h>
#include <ros.h>

// RGB ////////////////////////
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
// RGB ////////////////////////

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>

#define NORMALIZE(z) atan2(sin(z), cos(z))  // auxiliary function to normalize angle to the -pi, pi domain

// sending msgs
std_msgs::Float32 left_distance;
std_msgs::Float32 front_distance;
std_msgs::Float32 right_distance;
std_msgs::Bool line_front;
std_msgs::Bool line_back;
geometry_msgs::Pose2D pose;

// recieving msgs
std_msgs::UInt8MultiArray rgb_leds;
geometry_msgs::Twist cmd_velocity;
geometry_msgs::Pose2D initial_pose;

ros::NodeHandle  nh;

// RGB ////////////////////////
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define GRB_pin 10

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 2

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, GRB_pin, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second
// RGB ////////////////////////


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

//buzzer
const int buzzer_pin = 11;
// RGB LEDs
int Led1_R, Led1_G, Led1_B, Led2_R, Led2_G, Led2_B;


// task constants
const float counting_freq = 10.0;
const float left_pulses_correction = 1.00; // correction coeficient


int IR_to_mm(float IR_raw){
    // 320, 1.1 and 3.7 params found experimentally using curve fitting
    // returns distance in mm
    return (pow(320/IR_raw, 1.1) + 3.7)*10;
}


// 1.
float sense_L () {
    return IR_to_mm(map(analogRead(IR_L_pin), 0, 1023, 0, 100))*0.001;
}
// 2.
float sense_F () {
    return IR_to_mm(map(analogRead(IR_F_pin), 0, 1023, 0, 100))*0.001;
}
// 2.
float sense_R () {
    return IR_to_mm(map(analogRead(IR_R_pin), 0, 1023, 0, 100))*0.001;
}



// 3.
bool sense_line_F () {
    return (analogRead(line_F_pin)>500);
}
// 4.
bool sense_line_B () {
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
float x = 0;
float y = 0;
float theta = 0;

float x_init = 0.0;
float y_init = 0.0;
float theta_init = 0.0;


//displacement
float D;

// robot dimensions
const float b = 0.095; //m 95 mm 
const float r = 0.015; //m 15 mm

float VD = 0.0; // 0.08 max m/s
float WD = 0.0; // 2 max rad/s

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
    theta = atan2(sin(theta + w*delta_t), cos(theta + w*delta_t));
    x = x + v*delta_t * cos(theta); // should be *delta_t in seconds
    y = y + v*delta_t * sin(theta);
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
    float Ki = 16.0;
    float Kd = 4.0;

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
    else {
//        Serial.println("Speed is Out of limits !!!");
    }
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
//        Serial.print(" Unexpected combination of speeds ");
//        Serial.print(speed_left);
//        Serial.print(speed_right);
//        Serial.println();
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
//    delay(2150);
    stopFast();
    count_pulses(); // with given freq

//    Serial.print("pulses_num_left: ");
//    Serial.print(pulses_num_left);
//    Serial.println();
}


//
// ROS communication
//


// Subscribers

void rgb_leds_cb( const std_msgs::UInt8MultiArray& rgb_leds_msg){
    Led1_R = rgb_leds_msg.data[0];
    Led1_G = rgb_leds_msg.data[1];
    Led1_B = rgb_leds_msg.data[2];
    Led2_R = rgb_leds_msg.data[3];
    Led2_G = rgb_leds_msg.data[4];
    Led2_B = rgb_leds_msg.data[5];
}

void cmd_vel_cb( geometry_msgs::Twist& cmd_vel_msg){
    VD = cmd_vel_msg.linear.x; // m/s
    WD = cmd_vel_msg.angular.z; // rad/s
}


void set_pose_cb( geometry_msgs::Pose2D& set_pose_msg){
    // set x y theta to 0
    if (set_pose_msg.x != x_init || set_pose_msg.y != y_init || set_pose_msg.theta != theta_init){
      x_init = set_pose_msg.x;
      y_init = set_pose_msg.y;
      theta_init = set_pose_msg.theta;
      
      x = set_pose_msg.x;
      y = set_pose_msg.y;
      theta = set_pose_msg.theta;    
    }
}

ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub("/rgb_leds", rgb_leds_cb );

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", cmd_vel_cb );

ros::Subscriber<geometry_msgs::Pose2D> set_pose_sub("/set_pose", set_pose_cb );


// Publishers

std_msgs::Bool front_bool_msg;
std_msgs::Bool back_bool_msg;
ros::Publisher line_front_pub("/line_front", &front_bool_msg);
ros::Publisher line_back_pub("/line_back", &back_bool_msg);

std_msgs::Float32 left_float32_msg;
std_msgs::Float32 front_float32_msg;
std_msgs::Float32 right_float32_msg;
ros::Publisher left_distance_pub("/left_distance", &left_float32_msg);
ros::Publisher front_distance_pub("/front_distance", &front_float32_msg);
ros::Publisher right_distance_pub("/right_distance", &right_float32_msg);

geometry_msgs::Pose2D pose_msg;
ros::Publisher pose_pub("/pose", &pose_msg);

geometry_msgs::Twist curr_vel_msg;
ros::Publisher curr_vel_pub("/curr_vel", &curr_vel_msg);



void publish_all(){
  
    // get sensors data
    front_bool_msg.data = sense_line_F();
    back_bool_msg.data = sense_line_B();
    left_float32_msg.data = sense_L();
    front_float32_msg.data = sense_F();
    right_float32_msg.data = sense_R();
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = theta;
    curr_vel_msg.linear.x = v;
    curr_vel_msg.angular.z = w;
     
    // publish sensors data
    line_front_pub.publish( &front_bool_msg );
    line_back_pub.publish( &back_bool_msg );
    left_distance_pub.publish( &left_float32_msg );
    front_distance_pub.publish( &front_float32_msg );
    right_distance_pub.publish( &right_float32_msg );
    pose_pub.publish( &pose_msg );
    curr_vel_pub.publish( &curr_vel_msg );
}


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

    nh.initNode();

    nh.advertise(line_front_pub);
    nh.advertise(line_back_pub);
    nh.advertise(left_distance_pub);
    nh.advertise(front_distance_pub);
    nh.advertise(right_distance_pub);
    nh.advertise(pose_pub);
    nh.advertise(curr_vel_pub);
    
    nh.subscribe(rgb_leds_sub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(set_pose_sub);

//    Serial.begin(9600);
}

void loop() {

    curr_Millis=millis();
    
//    turnLeft();
    
    if (curr_Millis-prev_count_Millis >= 1000/counting_freq) {

        prev_count_Millis = curr_Millis;

        calc_pose2();
        
        publish_all();

        pid_controller();

        move_at(G[0], G[1]);

//        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//        pixels.setPixelColor(0, pixels.Color(0,150,0)); // Moderately bright green color.
//        pixels.setPixelColor(1, pixels.Color(0,150,0)); // Moderately bright green color.
//        pixels.show(); // This sends the updated pixel color to the hardware.  

    }

    
        nh.spinOnce();


//    Serial.print(w*100);
//    Serial.print(v*100);
//    Serial.println();
}






