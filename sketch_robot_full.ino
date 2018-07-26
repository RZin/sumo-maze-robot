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

// motor_right
#define AIN1 5 
#define AIN2 6  
#define PWMA 4 

// pulses counting vars
Encoder motorRight(3, 2);
long last_pulses_num_Right = 0;
long curr_pulses_num_Right;
int pulses_num_right;

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

int t = 0;

void encFunc() {t++;}

void loop() {
  test_movements();
}

