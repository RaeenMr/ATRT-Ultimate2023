// ALL RIGHT RESERVED FOR ATRT
int   buff[8];
int   counter, GY;
int   blocks;
float K_P=0, K_I=0, K_D=0, Heading, last_Heading;
int   angle_ball, direction_ball, distance_ball , x_ball , y_ball;
int   shr, shl, shb, dif;
int   shoot_sens,sensor[9];
int   b=4000;
bool  is_ball,Ball_In_Kicker = false;
bool  LDR_F,LDR_B,LDR_R,LDR_L;
int   LDR_SET_F = 0,LDR_SET_R = 0,LDR_SET_B = 0,LDR_SET_L = 0;
int   LCD_Print_Mode = 0;
bool  already_shooted = false;
int   out_cnt = 0, comeback_cnt = 0;
int   last_time;
int   v=220;
int   a = 0;
#define Walll_Distance  2540
#define Wallr_Distance  2700
#define back_Distance   1150
#define LDR_Sensitivity 300
/////////////////////ATRT 2
// #define x_robot 117
// #define y_robot 153
/////////////////////ATRT 1
#define x_robot 116
#define y_robot 154
// ///////////////////Robot 1
// #define x_robot 133
// #define y_robot 161 
// ///////////////////Robot 2
// #define x_robot 118
// #define y_robot 145 

void setup() {
  inti();
  set_ldr();
}

void loop() {
  AI_2();
}