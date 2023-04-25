//ALL RIGHT RESERVED FOR ATRT
#include <Wire.h>
#include <PixyI2C.h>
PixyI2C pixy;
#include <Adafruit_SH1106_STM32.h>
Adafruit_SH1106 display(-1);

void inti() {
  pinMode(PC13, OUTPUT);  // Built-in LED
  pinMode(PC14, OUTPUT);  // Spin
  pinMode(PC15, OUTPUT);  // Shoot
  // -------------------- Motor Settings (mohammad : tarif pin stm32)
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB12, OUTPUT);
  pinMode(PA8, PWM);
  pinMode(PB8, PWM);
  pinMode(PB7, PWM);
  pinMode(PB6, PWM);
  motor(0, 0, 0, 0);
  // -------------------- OLED Display Settings
  display.begin(0x2, 0x3C);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  // -------------------- Loading
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print("Loading");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.print(".");
  display.display();
  delay(500);
  display.setTextSize(1);
  // -------------------- Pixy Settings
  pixy.init();
  // -------------------- Serial Settings for GY-25
  Serial1.begin(115200);
  Serial1.write(0xA5);
  Serial1.write(0x54);
  delay(500);
  Serial1.write(0xA5);
  Serial1.write(0x51);
  // ------------------- PID
  last_time = millis();
}
void sensors() {
  // -------------------- Set Button
  if (digitalRead(PB5)) {
    digitalWrite(PC13, 0);
    if (LCD_Print_Mode == 0) {
      Serial1.write(0XA5);
      Serial1.write(0X55);
    } else if (LCD_Print_Mode == 1) set_ldr();
    while (digitalRead(PB5))
      ;
    digitalWrite(PC13, 1);
  }
  if (digitalRead(PB4)) {
    digitalWrite(PC13, 0);
    LCD_Print_Mode++;
    LCD_Print_Mode %= 2;
    while (digitalRead(PB4))
      ;
    digitalWrite(PC13, 1);
  }
  if (digitalRead(PA15)) {
    digitalWrite(PC13, 0);
    shoot_key();
    while (digitalRead(PA15))
      ;
    digitalWrite(PC13, 1);
  }
  // -------------------- GY-25 Read Data
  Serial1.write(0xA5);
  Serial1.write(0x51);
  while (true) {
    buff[counter] = Serial1.read();
    if (counter == 0 && buff[0] != 0xAA) break;
    counter++;
    if (counter == 8) {
      counter = 0;
      if (buff[0] == 0xAA && buff[7] == 0x55) {
        Heading = (int16_t)(buff[1] << 8 | buff[2]) / 100;
      }
    }
  }
  // -------------------- Pixy Read Data
  uint16_t blocks;
  blocks = pixy.getBlocks();
  is_ball = false;
  if (blocks) {
    for (int i = 0; i < blocks; i++) {
      if (pixy.blocks[i].signature == 1) {
        x_ball = pixy.blocks[i].y;
        y_ball = pixy.blocks[i].x;
        angle_ball = get_angle(x_ball, y_ball);
        direction_ball = get_direction(angle_ball);
        distance_ball = sqrt(pow(x_ball - x_robot, 2) + pow(y_ball - y_robot, 2));
        is_ball = true;
      }
    }
  }
  // -------------------- Sharp Read Data
  sensor[0] = analogRead(PA0);
  sensor[1] = analogRead(PA1);
  sensor[2] = analogRead(PA2);
  sensor[3] = LDR_SET_F - analogRead(PA3);
  sensor[4] = LDR_SET_B - analogRead(PA4);
  sensor[5] = LDR_SET_R - analogRead(PA5);
  sensor[6] = LDR_SET_L - analogRead(PA6);
  sensor[7] = analogRead(PA7);
  sensor[8] = analogRead(PA8);
  if (sensor[3] > LDR_Sensitivity) LDR_F = true;
  else LDR_F = false;
  if (sensor[4] > LDR_Sensitivity) LDR_B = true;
  else LDR_B = false;
  if (sensor[5] > LDR_Sensitivity) LDR_R = true;
  else LDR_R = false;
  if (sensor[6] > LDR_Sensitivity) LDR_L = true;
  else LDR_L = false;
  shb = sensor[1];
  shr = sensor[0];
  shl = sensor[2];
  dif = (shl - shr) / 10;
  // -------------------- Shoot Sensor Read Data
  shoot_sens = analogRead(PA7);
  if (shoot_sens > 2000) Ball_In_Kicker = true;
  else {
    Ball_In_Kicker = false;
    already_shooted = false;
  }
  // -------------------- PID Calculations
  K_P = Heading;
  if (millis() - last_time > 3000) {
    if (Heading < 3 && Heading > -3) {
      K_I = 0;
      K_D = 0;
    } else {
      K_I = (K_I + Heading) * 1.0;
      K_D = (Heading - last_Heading) / 1.0;
      if (K_I > 150) K_I = 150;
      if (K_D > 150) K_D = 150;
      if (K_I < -150) K_I = -150;
      if (K_D < -150) K_D = -150;
    }
    last_time = millis();
    last_Heading = Heading;
  }
  GY = 0.9 * K_P + 0.15 * K_I + 0.05 * K_D;
}
int get_angle(int x, int y) {
  int angle = atan2(y - y_robot, x - x_robot) * 180 / PI;
  angle += 90;
  if (angle > 360) angle -= 360;
  if (angle < 0) angle += 360;
  return angle;
}
int get_direction(int angle) {
  int direction;
  for (int i = 0; i < 16; i++) {
    if ((angle - 11.25 >= i * 22.5) && (angle - 11.25 < (i + 1) * 22.5)) direction = i + 1;
  }
  if (angle <= 11.25 || angle >= 348.5) direction = 0;
  return direction;
}
void print_all() {
  display.clearDisplay();
  if (LCD_Print_Mode == 0) {
    // --------------------- Pixy Print
    //    display.setCursor(0, 0);
    //    display.print("X:");
    //    display.println(x_ball);
    //    display.print("Y:");
    //    display.println(y_ball);
    //    display.print("ang:");
    //    display.println(angle_ball);
    //    display.print("dir:");
    //    display.println(direction_ball);
    //    display.print("dis:");
    //    display.println(distance_ball);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("X:");
    display.println(x_ball);
    display.print("Y:");
    display.println(y_ball);
    display.print("dir:");
    display.println(direction_ball);
    display.print("dis:");
    display.println(distance_ball);
    display.println("ang:");
    display.setTextSize(2);
    display.println(angle_ball);
  } else if (LCD_Print_Mode == 1) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    for (int i = 0; i < 9; i++) {
      display.print(i);
      display.print(" : ");
      display.println(sensor[i]);
    }
  }
  // --------------------- GY-25 Print Circle
  display.drawCircle(80, 32, 12, WHITE);
  display.drawLine(80 + sin(Heading * PI / 180) * 9, 32 - cos(Heading * PI / 180) * 9, 80 - sin(Heading * PI / 180) * 9, 32 + cos(Heading * PI / 180) * 9, WHITE);
  display.fillCircle(80 - sin(Heading * PI / 180) * 9, 32 + cos(Heading * PI / 180) * 9, 2, WHITE);
  // --------------------- Ball Print Circle
  if (Ball_In_Kicker)
    display.fillCircle(80, 32 - 16, 3, WHITE);
  else if (is_ball)
    display.fillCircle(80 + sin(angle_ball * PI / 180) * 20, 32 - cos(angle_ball * PI / 180) * 20, 3, WHITE);
  // --------------------- Out LDR
  // if(shr > Wallr_Distance) display.fillRect(105, 27, 3, 10, WHITE);
  // if(shl > Walll_Distance) display.fillRect(53, 27, 3, 10, WHITE);
  // if(shb > back_Distance ) display.fillRect(75, 57, 10, 3, WHITE);
  if (LDR_F) display.fillRect(80 - 2, 32 - 16, 4, 6, WHITE);
  if (LDR_B) display.fillRect(80 - 2, 32 + 10, 4, 6, WHITE);
  if (LDR_R) display.fillRect(80 + 10, 32 - 2, 6, 4, WHITE);
  if (LDR_L) display.fillRect(80 - 16, 32 - 2, 6, 4, WHITE);
  display.display();
}
void set_ldr() {
  LDR_SET_F = analogRead(PA3);
  LDR_SET_R = analogRead(PA4);
  LDR_SET_B = analogRead(PA5);
  LDR_SET_L = analogRead(PA6);
}
void motor(int ML1, int ML2, int MR2, int MR1) {
  ML1 += GY;
  ML2 += GY;
  MR2 += GY;
  MR1 += GY;
  ML1 *= 235;
  ML2 *= 255;
  MR2 *= 255;
  MR1 *= 235;
  if (ML1 > 65535) ML1 = 65535;
  if (ML2 > 65535) ML2 = 65535;
  if (MR2 > 65535) MR2 = 65535;
  if (MR1 > 65535) MR1 = 65535;
  if (ML1 < -65535) ML1 = -65535;
  if (ML2 < -65535) ML2 = -65535;
  if (MR2 < -65535) MR2 = -65535;
  if (MR1 < -65535) MR1 = -65535;



  if (ML1 > 0) {
    digitalWrite(PB15, 0);
    pwmWrite(PA8, ML1);
  } else {
    digitalWrite(PB15, 1);
    pwmWrite(PA8, ML1 + 65535);
  }
  if (ML2 > 0) {
    digitalWrite(PB14, 0);
    pwmWrite(PB8, ML2);
  } else {
    digitalWrite(PB14, 1);
    pwmWrite(PB8, ML2 + 65535);
  }
  if (MR2 > 0) {
    digitalWrite(PB13, 0);
    pwmWrite(PB7, MR2);
  } else {
    digitalWrite(PB13, 1);
    pwmWrite(PB7, MR2 + 65535);
  }
  if (MR1 > 0) {
    digitalWrite(PB12, 0);
    pwmWrite(PB6, MR1);
  } else {
    digitalWrite(PB12, 1);
    pwmWrite(PB6, MR1 + 65535);
  }
}
void move(int direction) {
  if (direction == 0) motor(v, v, -v, -v);
  if (direction == 1) motor(v, v / 2, -v, -v / 2);
  if (direction == 2) motor(v, 0, -v, 0);
  if (direction == 3) motor(v, -v / 2, -v, v / 2);
  if (direction == 4) motor(v, -v, -v, v);
  if (direction == 5) motor(v / 2, -v, -v / 2, v);
  if (direction == 6) motor(0, -v, 0, v);
  if (direction == 7) motor(-v / 2, -v, v / 2, v);
  if (direction == 8) motor(-v, -v, v, v);
  if (direction == 9) motor(-v, -v / 2, v, v / 2);
  if (direction == 10) motor(-v, 0, v, 0);
  if (direction == 11) motor(-v, v / 2, v, -v / 2);
  if (direction == 12) motor(-v, v, v, -v);
  if (direction == 13) motor(-v / 2, v, v / 2, -v);
  if (direction == 14) motor(0, v, 0, -v);
  if (direction == 15) motor(v / 2, v, -v / 2, -v);
}
void out_sharp() {
  if (shr > Wallr_Distance) {
    while (direction_ball < 8 && direction_ball > 0 && is_ball) {
      sensors();
      print_all();
      v = 255;
      if (shr > Wallr_Distance + 300) move(12);
      else stop();
    }
  }
  if (shl > Walll_Distance) {
    while (direction_ball > 8 && is_ball) {
      sensors();
      print_all();
      v = 255;
      if (shl > Walll_Distance + 300) move(4);
      else stop();
    }
  }
  ////////////////////////////////////////////

  if (shb > b) {
    while ((direction_ball >= 1 && direction_ball <= 9) && is_ball) {
      sensors();
      print_all();
      v = 255;
      if (shb > b + 200) move(0);
      else stop();
    }
  }
}
void stop() {
  motor(0, 0, 0, 0);
}
void movexy(int x, int y) {
  motor((x + y) / 2, (y - x) / 2, (-x - y) / 2, (x - y) / 2);
}
void moveAngle(int angle) {
  if (angle > 360) angle -= 360;
  if (angle < 0) angle += 360;
  movexy(v * sin(angle * PI / 180), v * cos(angle * PI / 180));
}
void come_back() {
  v = 170;
  if (abs(dif) > 60) motor(-v / 2 + dif, -v / 2 - dif, v / 2 - dif, v / 2 + dif);
  else if (shb < back_Distance) motor(-v + dif, -v - dif, v - dif, v + dif);
  else motor(0, 0, 0, 0);
}
void AI_1() {
  sensors();
  print_all();
  //out_sharp();
  if (is_ball) {
    if (distance_ball > 70) {
      v = 200;
      move(direction_ball);
    } else {
      v = 130;
      if (direction_ball == 0) move(direction_ball);
      else if (direction_ball == 1) move(direction_ball + 1);
      else if (direction_ball == 15) move(direction_ball - 1);
      else if (direction_ball <= 3) move(direction_ball + 2);
      else if (direction_ball <= 8) move(direction_ball + 3);
      else if (direction_ball <= 13) move(direction_ball - 3);
      else move(direction_ball - 2);
    }
  } else {
    come_back();
  }
}
void AI_2() {
  sensors();
  print_all();
  out();
  //out_sharp();
  if (Ball_In_Kicker) {
    v = 160;
    //move(0);
    shoot();
    comeback_cnt = 0;
  } else if (is_ball) {
    if (distance_ball > 50) {
      v = 200;
      moveAngle(angle_ball);
    } else {
      ///////// 5 _____355 /////////////////har kodam +10
      v = 200;
      if (angle_ball < 12 || angle_ball > 356) moveAngle(angle_ball);
      else if (angle_ball < 20) moveAngle(angle_ball + 35);
      else if (angle_ball < 90) moveAngle(angle_ball + 55);
      else if (angle_ball < 180) moveAngle(angle_ball + 85);
      else if (angle_ball < 270) moveAngle(angle_ball - 85);
      else if (angle_ball < 340) moveAngle(angle_ball - 55);
      else moveAngle(angle_ball - 35);
    }
    comeback_cnt = 0;
  } else if (comeback_cnt < 65) {
    come_back();
    comeback_cnt++;
  } else stop();
}
void shoot_key() {
  digitalWrite(PC15, 1);
  delay(70);
  digitalWrite(PC15, 0);
  delay(200);
}
void shoot() {
  if (already_shooted) return;
  if (Ball_In_Kicker) {
    digitalWrite(PC15, 1);
    delay(70);
    digitalWrite(PC15, 0);
    delay(200);
    already_shooted = true;
  }
}
void moveForSec(int dir, int sec) {
  for (int i = 0; i < sec; i++) {
    sensors();
    print_all();
    move(dir);
  }
}
void moveInside() {
  if (LDR_R && LDR_F) move(10);
  if (LDR_L && LDR_F) move(6);
  if (LDR_R && LDR_B) move(14);
  if (LDR_L && LDR_B) move(2);
  if (LDR_F) move(8);
  if (LDR_R) move(12);
  if (LDR_B) move(0);
  if (LDR_L) move(4);
}
void out() {
  out_cnt = 0;
  if (LDR_F && LDR_R) {
    v = 170;
    moveForSec(10, 5);
    while ((angle_ball < 170 || angle_ball > 280) && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_F && LDR_L) {
    v = 170;
    moveForSec(6, 5);
    while ((angle_ball < 80 || angle_ball > 190) && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B && LDR_L) {
    v = 170;
    moveForSec(2, 5);
    while (angle_ball > 100 && angle_ball < 350 && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B && LDR_R) {
    v = 170;
    moveForSec(14, 5);
    while (angle_ball > 10 && angle_ball < 260 && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_F) {
    v = 170;
    moveForSec(8, 11);
    while ((angle_ball < 90 || angle_ball > 270) && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_R) {
    v = 170;
    moveForSec(12, 10);
    while (angle_ball < 180 && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_B) {
    v = 170;
    moveForSec(0, 11);
    while (angle_ball > 90 && angle_ball < 270 && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  } else if (LDR_L) {
    v = 170;
    moveForSec(4, 10);
    while (angle_ball > 180 && is_ball && out_cnt < 100) {
      sensors();
      print_all();
      v = 170;
      if (LDR_R || LDR_F || LDR_B || LDR_L) moveInside();
      else stop();
      out_cnt++;
    }
  }
}
void shootshift() {
  v = 200;
  if (angle_ball < Wallr_Distance)   move(4);
  else if (angle_ball >= 220)        shoot();
  else if (angle_ball <  170)        shoot();
  else if (angle_ball >  130)        shoot();
  else if (angle_ball <  130)        shoot();
  else                           come_back();
  if (angle_ball < Walll_Distance)  move(12);
  else if (angle_ball <= 270)        shoot();
  else if (angle_ball <  180)        shoot();
  else if (angle_ball >  130)        shoot();
  else if (angle_ball <  130)        shoot();
  else                           come_back();
}