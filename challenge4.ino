
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

float VD = 1.5;
float WD = 1.0;

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

float wLd_curr= 0;
float wRd_curr= 0;

const float b=20; //cm
const float r=5; // cm
const int C=100; // not sure 

void encUpdate(int t) {
  NL = N[t][0]; // read
  NR = N[t][1];
}

//const frequency = 10;
const int delta_t = 100;

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
  x[t+1] = x[t] + D * cos(theta[t]);
  y[t+1] = y[t] + D * sin(theta[t]); 
  theta[t+1] = theta[t] + (DR-DL)/b;

  // get v_curr, w_curr
  cmd_vel();
  
  // get wL_curr, wR_curr
  vel_to_wheels(v_curr, w_curr);
}

void calc_pose2(){
    // get NL NR
    count_pulses();

}

// gain constants
float GL[10] = {0};
float GR[10] = {0};


void PID (){
  // define errors
  float ep[10][2] = 
{{wLd[0]- wL[0]}, 
 {wRd[0]- wR[0]}};

  float ei[10][2] = 
{{0}, 
 {0}};

  float ed[10][2] = 
{{0}, 
 {0}};

  // coefs Kp
  float Kp = 0.5;
  float Ki = 0;
  float Kd = 0;

  // update errors
  while (t < 9){
    // get wL, wR
    
    vel_to_wheels(V, W);
    // get wLd, wRd
    get_wheels_speed_desired(VD, WD);
    
    // left proportional
    ep[t+1][0] = {wLd[t+1] - wL[t+1]};
     // right proportional
    ep[t+1][1] = {wRd[t+1] - wR[t+1]}; 
    
    // left integral
    ei[t+1][0] = (ep[t+1][0] + ep[t][0])*t; // delta_t
     // right integral
    ei[t+1][1] = (ep[t+1][1] + ep[t][1])*t;
    {wLd[t+1]-wL[t+1], 
    
    // left derivative
    ed[0][t+1] = (ep[t+1][0] - ep[t][0])/ t;
     // right derivative
    ed[1][t+1] = (ep[t+1][1] - ep[t][1])/ t;

    // get the gain 
    GL[t+1] = Kp*ep[t+1][0] + Ki*ei[t+1][0] + Kd*ed[t+1][0];
    GR[t+1] = Kp*ep[t+1][1] + Ki*ei[t+1][1] + Kd*ed[t+1][1];
    
    t++;}
  }
}

void loop() {
  delay(1000);
  encUpdate(t);
  posUpdate(NL, NR, r, b, C);

  Serial.println("x");
  Serial.println(x[t]);

  Serial.println("y");
  Serial.println(y[t]);  

  Serial.println("theta");
  Serial.println(theta[t]);

  Serial.println("theta");
  Serial.println(theta[t]);

  Serial.println("theta");
  Serial.println(theta[t]);

  if (t < 9){t++;}
  else {
    Serial.println("done !");
}

}





