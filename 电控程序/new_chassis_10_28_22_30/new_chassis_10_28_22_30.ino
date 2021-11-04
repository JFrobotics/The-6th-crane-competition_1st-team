/*
           车头

     8              4
     9              5
          6    7
  车内空间分配命名上层小写abc，下层大写ABC
    车头
  a  |b  |  c
  A  |B  |  C
  对应应放置箱子为
  R_T|R_C|R_S
  B_T|B_C|B_S

  取物装置对应三个位置命名A B C
  2006电机正数往C移动 负数往A移动
  10.26 17.36 测完abc 限位开关
  20.26 23.23改为先识别后夹取
  20.27 13.06 改取货运动时夹爪位置
  20.27 13.06 改全程运动时夹爪位置
  向左移动  ANGLE = -85;
  ANGLE_VELOCITY = -9; 走直
  向右移动  ANGLE = -85;
  ANGLE_VELOCITY = -9; 走直
  向前移动
  ANGLE = 1;
  ANGLE_VELOCITY = -20;走直
  向右移动
    ANGLE = 90.5;
  ANGLE_VELOCITY = 11;走直

  左巡线成功参数
  1000/200
  右巡线成功参数
  1000/200
  stage0 stage1已经调完
  22.03stage2完成

*/
#include <mcp_can.h>
#include <SPI.h>
const int SPI_CS_PIN = 53; //设置53号引脚为cs片选
MCP_CAN CAN(SPI_CS_PIN);

int SPEED = 0, ANGLE = 0, ANGLE_VELOCITY = 0, MOTOR_2006 = 0;                      //底盘运动速度，角度，角速度  2006电机速度
int D4state = 0, D5state = 0, D6state = 0, D7state = 0, D8state = 0, D9state = 0; //存放光电模块状态信息
int D22state = 0, D24state = 0, D26state = 0;                                     //存放行程开关状态
int D28state = 0;                                                                 //存放磁性开关状态
int stage = 5;                                                                     //阶段数
int grip_position = 0;                                                             //夹爪目标位置标志位 1 2 3 11 22 33
int grip_now_position = 0;                                                         //夹爪现在位置标志位 1 2 3 对应A B C
int start_box_1 = 1;
int put_box_num = 0;

////////////////////////////////////////////////////////////////////////////////////////////
void receive_from_pi();
void right_patrol();
void STOP();
void move_forward(int move_forward_speed);
void box_2_6();

//////////////////////////////////////////////////////////////////////////////////////////////
int a = 0;
void setup()
{

  CAN.begin(CAN_1000KBPS, MCP_8MHz);

  Serial.begin(115200);
  Serial2.begin(115200); //连接蓝牙与云台通讯

  pinMode(4, INPUT); //光电循迹
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(22, INPUT_PULLUP); //行程开关A位 COM接负极 挡住输出低电平
  pinMode(24, INPUT); //行程开关B位 挡住输出高电平
  pinMode(26, INPUT_PULLUP); //行程开关C位 COM接负极 挡住输出低电平

  pinMode(28, INPUT); //抬升气缸磁性开关感应

  pinMode(30, OUTPUT); //夹取气缸
  pinMode(32, OUTPUT); //摆杆气缸
  pinMode(34, OUTPUT); //抬升气缸抬起
  pinMode(36, OUTPUT); //抬升气缸下降推货气缸
  pinMode(38, OUTPUT); //推出气缸
  pinMode(40, OUTPUT); //顶气缸
  pinMode(42, OUTPUT);  //电磁锁A位
  pinMode(44, OUTPUT);  //电磁锁B位
  pinMode(46, OUTPUT);  //电磁锁C位


  digitalWrite(30, LOW);
  digitalWrite(32, HIGH);
  digitalWrite(34, LOW);
  digitalWrite(36, HIGH);//初始下降状态
  digitalWrite(38, LOW);
  digitalWrite(40, HIGH);

  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);

  stage = 5;
  grip_now_position = 0;
}

void loop()
{ /*delay(2000);
    digitalWrite(40, LOW); //死点气缸下降
    digitalWrite(32, LOW); //摆杆放下
    delay(1000);
    do
    {

    D28state = digitalRead(28);
    digitalWrite(34, HIGH);

    digitalWrite(36, LOW); //抬升气缸抬起
    } while (D28state == 0);
    digitalWrite(34, LOW);
    digitalWrite(36, LOW); //抬升气缸停住
    /*delay(1000);
    digitalWrite(34, LOW);
    digitalWrite(36, HIGH);
    while(1);*/
//move_left(1000);
 /* left_patrol();
  if (D8state == 1 && D9state == 0)
  {
    delay(1000);
  }*/

  //STAR_A();
  STAR_B();
  //test_guangdian();
  //back_patrol();
  // move_back(1000);
  /*do
       {
         clamp_move_to_C();
       }
       while (grip_now_position != 3);*/
}
void test_box2_6()
{
  grip_now_position = 0;
  do
  {
    clamp_move_to_C();
  }
  while (grip_now_position != 3);

  do
  {
    clamp_move_to_B();
  }
  while (grip_now_position != 2);
  Serial.println("到达B");
  clamp_pick();           //到位后夹取到平台上
  receive_from_pi();      //此处有while接收到信号并判断夹爪目标位置后跳出
  drop_to_car_position(); //有switch /while /grip_now_position到相应值后完成动作后跳出
}
void test_guangdian()
{
  D4state = digitalRead(4);
  D5state = digitalRead(5);
  D6state = digitalRead(6);
  D7state = digitalRead(7);
  D8state = digitalRead(8);
  D9state = digitalRead(9);
  //Serial.print("D4state"); Serial.println(D4state);
  //   Serial.print("D5state"); Serial.println(D5state);
  Serial.print("D6state"); Serial.println(D6state);
  Serial.print("D7state"); Serial.println(D7state);
  //Serial.print("D 8 state"); Serial.println(D8state);
  //Serial.print("D 9 state"); Serial.println(D9state);
  delay(100);

}
void test_to_a()
{
  do
  {
    clamp_move_to_A();
  } while (grip_now_position != 1);
  delay(2000);

  do
  {
    clamp_move_to_B();
  } while (grip_now_position != 2);
  delay(2000);
  do
  {
    clamp_move_to_C();
  } while (grip_now_position != 3);
  motor2006_move(0);

}
void test_2006()
{
  motor2006_move(-4000);
  delay(2000);
  motor2006_move(0);
  while (1);
}
void test() {

  /* digitalWrite(30, HIGH);
    delay(500);
    digitalWrite(32, HIGH);
     delay(500);
    digitalWrite(34, HIGH);
     delay(500);
    digitalWrite(36, HIGH);
     delay(500);
    digitalWrite(38, HIGH);
     delay(500);
    digitalWrite(40, HIGH);
     delay(500);
    digitalWrite(42, HIGH);
     delay(500);
    digitalWrite(44, HIGH);
     delay(500);
    digitalWrite(46, HIGH);
     delay(500);*/
  /*delay(3000);
    digitalWrite(30, LOW);
    digitalWrite(32, LOW);
    digitalWrite(34, LOW);
    digitalWrite(36, LOW);
    digitalWrite(38, LOW);
    digitalWrite(40, LOW);
    digitalWrite(42, LOW);
    digitalWrite(44, LOW);
    digitalWrite(46, LOW);
    delay(3000);*/
  /*digitalWrite(40, LOW); //死点气缸下降
    digitalWrite(32, LOW); //摆杆放下
    delay(5000);
    digitalWrite(30, HIGH); //夹取
    delay(1000);
    digitalWrite(40, HIGH); //死点气缸上升
    delay(1000);
    digitalWrite(32, HIGH); //摆杆抬起
    delay(5000);*/
  /*digitalWrite(30, LOW); //夹爪松开
    delay(3000);
    digitalWrite(30, HIGH); //夹爪合上
    delay(3000);*/
  /*delay(5000);
    digitalWrite(40, LOW); //死点气缸下降
    delay(1000);
    digitalWrite(32, LOW); //摆杆放下
    delay(1000);*/

  /*  delay(2000);
    digitalWrite(32, HIGH);
    digitalWrite(40, HIGH);*/

  /*  delay(3000);
     motor2006_move(-3000);
      delay(2000);
    motor2006_move(3000);
    delay(2000);
    motor2006_move(0);*/
  move_forward(500);
  delay(8000);
  move_back(500);
  delay(8000);
  move_left(500);
  delay(8000);
  move_right(500);
  delay(8000);
  // motor2006_move(0);
  //delay(1000);
  STOP();
  //motor2006_move(5000);
  //delay(5000);*/

  while (1);
}
void test_case_a()
{
  clamp_pick();
  grip_now_position = 0;
  do
  {
    clamp_move_to_A();
  } while (grip_now_position != 1);

  do
  {
    D28state = digitalRead(28);
    digitalWrite(34, HIGH);
    digitalWrite(36, LOW); //抬升气缸抬起
    Serial.print("D28state"); Serial.println(D28state);
  } while (D28state == 0);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW); //抬升气缸停住
  delay(200);
  digitalWrite(38, HIGH); //推出气缸推出
  delay(1000);
  digitalWrite(38, LOW); //推出气缸收回
  digitalWrite(34, LOW);
  digitalWrite(36, HIGH); //抬升气缸下降
}
void test_case_c()
{
  clamp_pick();
  grip_now_position = 0;
  do
  {
    clamp_move_to_C();
  } while (grip_now_position != 3);

  do
  {
    D28state = digitalRead(28);
    digitalWrite(34, HIGH);
    digitalWrite(36, LOW); //抬升气缸抬起
    Serial.print("D28state"); Serial.println(D28state);
  } while (D28state == 0);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW); //抬升气缸停住
  delay(200);
  digitalWrite(38, HIGH); //推出气缸推出
  delay(1000);
  digitalWrite(38, LOW); //推出气缸收回
  digitalWrite(34, LOW);
  digitalWrite(36, HIGH); //抬升气缸下降
}
void test_b()
{
  do
  {
    D28state = digitalRead(28);
    digitalWrite(34, HIGH);
    digitalWrite(36, LOW); //抬升气缸抬起
  } while (D28state == 0);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW); //抬升气缸停住
  delay(200);
  digitalWrite(38, HIGH); //推出气缸推出
  delay(1000);
  digitalWrite(38, LOW); //推出气缸收回
  digitalWrite(34, LOW);
  digitalWrite(36, HIGH); //抬升气缸下降

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  can控制底盘运动及摩擦轮2006电机子函数
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*底盘运动前后左右*/
void send_move_data(uint16_t move_speed, int16_t move_angle, int16_t angular_velocity, int16_t motor2006)
{
  uint8_t data[8];
  data[0] = (move_speed >> 8) & 0xFF;
  data[1] = move_speed & 0xFF;
  data[2] = (move_angle >> 8) & 0xFF;
  data[3] = move_angle & 0xFF;
  data[4] = (angular_velocity >> 8) & 0xFF;
  data[5] = angular_velocity & 0xFF;
  data[6] = (motor2006 >> 8) & 0xFF;
  data[7] = motor2006 & 0xFF;
  CAN.sendMsgBuf(0x208, 0, 8, data);
}

void move_forward(int move_forward_speed)
{
  SPEED = move_forward_speed;
  ANGLE = 1;
  ANGLE_VELOCITY = -20;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //前进
  Serial.println("前进");
}
void move_back(int move_back_speed)
{
  SPEED = move_back_speed;
  ANGLE = -180;
  ANGLE_VELOCITY = -40;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //后退
  Serial.println("后退");
}
void move_left(int move_left_speed)
{
  SPEED = move_left_speed;
  ANGLE = -85;
  ANGLE_VELOCITY = 100;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向左
  Serial.println("横向左");
}
void move_right(int move_right_speed)
{
  SPEED = move_right_speed;
  ANGLE = 90.5;
  ANGLE_VELOCITY = 11;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向右
  Serial.println("横向右");
}
void STOP()
{
  SPEED = 0;
  ANGLE = 0;
  ANGLE_VELOCITY = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006);
  Serial.println("STOP");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/*底盘运动前后左右行进旋转*/
void move_forward_left(int move_forward_speed, int move_forward_turn_left_speed)
{
  SPEED = move_forward_speed;
  ANGLE = 0;
  ANGLE_VELOCITY = -move_forward_turn_left_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //前进左转
  Serial.println("前进逆时针/左转");
}
void move_forward_right(int move_forward_speed, int move_forward_turn_right_speed)
{
  SPEED = move_forward_speed;
  ANGLE = 0;
  ANGLE_VELOCITY = move_forward_turn_right_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //前进右转
  Serial.println("前进顺时针/右转");
}
void move_back_left(int move_back_speed, int move_back_turn_speed)
{
  SPEED = move_back_speed;
  ANGLE = 180;
  ANGLE_VELOCITY = -move_back_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //后退左转
  Serial.println("后退逆时针/左转");
}
void move_back_right(int move_back_speed, int move_back_turn_speed)
{
  SPEED = move_back_speed;
  ANGLE = 180;
  ANGLE_VELOCITY = move_back_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //后退右转
  Serial.println("后退顺时针/右转");
}
void move_left_left(int move_left_speed, int move_left_turn_speed)
{
  SPEED = move_left_speed;
  ANGLE = -85;
  ANGLE_VELOCITY = -move_left_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向左左转
  Serial.println("横向左逆时针/左转");
}
void move_left_right(int move_left_speed, int move_left_turn_speed)
{
  SPEED = move_left_speed;
  ANGLE = -85;
  ANGLE_VELOCITY = move_left_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向左右转
  Serial.println("横向左顺时针/右转");
}
void move_right_left(int move_right_speed, int move_right_turn_speed)
{
  SPEED = move_right_speed;
  ANGLE = 90;
  ANGLE_VELOCITY = -move_right_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向右左转
  Serial.println("横向右逆时针/左转");
}
void move_right_right(int move_right_speed, int move_right_turn_speed)
{
  SPEED = move_right_speed;
  ANGLE = 90;
  ANGLE_VELOCITY = move_right_turn_speed;
  MOTOR_2006 = 0;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向右右转
  Serial.println("横向右顺时针/右转");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*2006电机运动*/
void motor2006_move(int motor_speed)
{
  MOTOR_2006 = motor_speed;
  send_move_data(SPEED, ANGLE, ANGLE_VELOCITY, MOTOR_2006); //横向右右转
  if (MOTOR_2006 > 0)
  {
    Serial.println("2006逆时针to C");
  }
  else if (MOTOR_2006 < 0)
  {
    Serial.println("2006顺时针to A");
  }
  else
  {
    Serial.println("2006停止");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*巡线子函数*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void left_patrol()
{
  D8state = digitalRead(8);
  D9state = digitalRead(9);
  if (D8state == 0 && D9state == 0)
  { move_left(1000);


  }
  if (D8state == 1 && D9state == 0)
  { move_left_right(1000, 300);//1000/100 //1000/200
    Serial.print("D 8 state=="); Serial.println(D8state);
    Serial.print("D 9 state=="); Serial.println(D9state);
    Serial.println("平移左 右转");
  }
  if (D8state == 0 && D9state == 1)
  { move_left_left(1000, 300); //1000/400  //1000/200
    Serial.print("D 9 state=="); Serial.println(D9state);
    Serial.println("平移左 左转");
  }
}
void right_patrol()
{
  D4state = digitalRead(4);
  D5state = digitalRead(5);
  Serial.print("D 4 state=="); Serial.print(D4state); Serial.print("  D 5 state=="); Serial.println(D5state);
  if (D4state == 0 && D5state == 0)
    move_right(1000);
  if (D4state == 1 && D5state == 0)
    move_right_left(1000, 200);
  if (D4state == 0 && D5state == 1)
    move_right_right(1000, 200);
}
void back_patrol()
{
  D6state = digitalRead(6);
  D7state = digitalRead(7);

  if (D6state == 0 && D7state == 0)
    move_back(1000);
  if (D6state == 1 && D7state == 0)
    move_back_right(1000, 250);
  if (D6state == 0 && D7state == 1)
    move_back_left(1000, 250);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  综合型函数
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////

void receive_from_pi()
{
  char readSrial2 = '\0';
  int recieved = 0;
  while (Serial2.read() >= 0)
  {
  }
  do
  {
    Serial.print("等待接收pi信号");
    if (Serial2.available())
    {
      readSrial2 = Serial2.read();
      Serial.println();
      Serial.print("接收到信号:");
      Serial.println(readSrial2);
      if (readSrial2 == 'A' || readSrial2 == 'B' || readSrial2 == 'C' || readSrial2 == 'a' || readSrial2 == 'b' || readSrial2 == 'c')
        sorting(readSrial2);
      recieved = 1;
    }
  } while (recieved != 1);
  Serial.print("readSrial2=="); Serial.println(readSrial2);

}
void sorting(char receive)
{
  switch (receive)
  {
    case 'A':
      grip_position = 1;
      Serial.print("grip_position=="); Serial.println(grip_position);

      break;
    case 'B':
      grip_position = 2;
      Serial.print("grip_position=="); Serial.println(grip_position);
      break;
    case 'C':
      grip_position = 3;
      Serial.print("grip_position=="); Serial.println(grip_position);
      break;
    case 'a':
      grip_position = 11;
      Serial.print("grip_position=="); Serial.println(grip_position);
      break;
    case 'b':
      grip_position = 22;
      Serial.print("grip_position=="); Serial.println(grip_position);
      break;
    case 'c':
      grip_position = 33;
      Serial.print("grip_position=="); Serial.println(grip_position);
      break;
    default:
      break;
  }
}
void clamp_pick()
{
  motor2006_move(0);
  digitalWrite(40, LOW); //死点气缸下降
  digitalWrite(32, LOW); //摆杆放下
  delay(2500);
  digitalWrite(30, HIGH); //夹取
  delay(1000);
  digitalWrite(40, HIGH); //死点气缸上升
  delay(300);
  digitalWrite(32, HIGH); //摆杆抬起
  delay(3000);
  digitalWrite(30, LOW); //夹爪松开
  delay(1000);

}
void clamp_setup()
{
  digitalWrite(40, HIGH); //死点气缸上升
  delay(300);
  digitalWrite(32, HIGH); //摆杆抬起
  delay(2500);
  digitalWrite(30, LOW); //夹爪松开
  delay(500);


}
void drop_to_car_position()
{
  switch (grip_position)
  {
    case 1:
      Serial.println("drop_to_car_position case 1");
      do
      {
        clamp_move_to_A();
      } while (grip_now_position != 1);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      do
      {
        D28state = digitalRead(28);
        digitalWrite(34, HIGH);
        digitalWrite(36, LOW); //抬升气缸抬起
      } while (D28state == 0);
      digitalWrite(34, LOW);
      digitalWrite(36, LOW); //抬升气缸停住
      delay(200);
      digitalWrite(38, HIGH); //推出气缸推出
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;
    case 2:
      Serial.println("drop_to_car_position case 2");
      do
      {
        clamp_move_to_B();
      } while (grip_now_position != 2);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      do
      {
        D28state = digitalRead(28);
        digitalWrite(34, HIGH);
        digitalWrite(36, LOW); //抬升气缸抬起
      } while (D28state == 0);
      digitalWrite(34, LOW);
      digitalWrite(36, LOW); //抬升气缸停住
      delay(200);
      digitalWrite(38, HIGH); //推出气缸推出
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;
    case 3:
      Serial.println("drop_to_car_position case 3");
      do
      {
        clamp_move_to_C();
      } while (grip_now_position != 3);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      do
      {
        D28state = digitalRead(28);
        digitalWrite(34, HIGH);
        digitalWrite(36, LOW); //抬升气缸抬起
      } while (D28state == 0);
      digitalWrite(34, LOW);
      digitalWrite(36, LOW); //抬升气缸停住
      delay(200);
      digitalWrite(38, HIGH); //推出气缸推出
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;
    case 11:
      Serial.println("drop_to_car_position case 11");
      do
      {
        clamp_move_to_A();
      } while (grip_now_position != 1);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      digitalWrite(34, HIGH);
      digitalWrite(36, LOW); //抬升气缸抬起
      Serial.println("case 11抬升气缸抬起");
      delay(2500);
      digitalWrite(38, HIGH); //推出气缸推出
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;
    case 22:
      Serial.println("drop_to_car_position case 22");
      do
      {
        clamp_move_to_B();
      } while (grip_now_position != 2);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      digitalWrite(34, HIGH);
      digitalWrite(36, LOW); //抬升气缸抬起
      Serial.println("case 22抬升气缸抬起");
      delay(2500);
      digitalWrite(38, HIGH); //推出气缸推出
      Serial.println("推出气缸推出");
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;
    case 33:
      Serial.println("drop_to_car_position case 33");
      do
      {
        clamp_move_to_C();
      } while (grip_now_position != 3);
      motor2006_move(0);
      digitalWrite(40, LOW); //死点气缸下降
      digitalWrite(32, LOW); //摆杆放下
      delay(1000);
      digitalWrite(34, HIGH);
      digitalWrite(36, LOW); //抬升气缸抬起
      Serial.println("case 33抬升气缸抬起");
      delay(2500);
      digitalWrite(38, HIGH); //推出气缸推出
      delay(1000);
      digitalWrite(38, LOW); //推出气缸收回
      digitalWrite(34, LOW);
      digitalWrite(36, HIGH); //抬升气缸下降
      delay(1000);
      clamp_setup();
      break;

    default:
      break;
  }
}
void clamp_move_to_A() //电b机正转
{
  D22state = digitalRead(22);
  Serial.println("clamp_move_to_A");
  if (D22state == 0)
  {
    grip_now_position = 1;
    motor2006_move(0);
    grip_now_position = 1;
  }
  else
  {
    motor2006_move(-5000);
  }
}
void clamp_move_to_C() //电机反转
{
  D26state = digitalRead(26);
  Serial.print("D26state=="); Serial.println(D26state);
  if (D26state == 0)
  {
    motor2006_move(0);
    grip_now_position = 3;
  }
  else
  {
    motor2006_move(5000);
  }
}
void clamp_move_to_B() //中间位
{
  D24state = digitalRead(24);
  Serial.print("D24state=="); Serial.println(D24state);
  if (D24state == 1)
  {
    motor2006_move(0);
    grip_now_position = 2;
  }
  else
  {
    if (grip_now_position == 1)
    {
      motor2006_move(5000);
    }
    else if (grip_now_position == 3)
    {
      motor2006_move(-5000);
    }
  }
}
void box_1() //此函数不需循环进入
{
  grip_now_position = 0;
  do
  {
    clamp_move_to_C();
  }
  while (grip_now_position != 3);
  Serial.println("在box_1中");

  clamp_pick();           //此处有延时
  receive_from_pi();      //此处有while接收到信号并判断夹爪目标位置后跳出
  drop_to_car_position(); //有while grip_now_position到相应值后完成动作后跳出
  start_box_1 = 0;
  Serial.println("box_1结束");
}
void box_2_6()
{
  int left_black_line = 0;
  int finish_box_num = 1;
  for (finish_box_num = 1; finish_box_num < 6; finish_box_num++)
  {
    Serial.print("finish_box_num");
    Serial.println(finish_box_num);
    left_black_line = 0;
    grip_now_position = 0;
    Serial.print("grip_now_position=="); Serial.println(grip_now_position);
    do
    {
      Serial.println("走前归位到C");
      clamp_move_to_C();
    }
    while (grip_now_position != 3);//左移之前要移到从位


    move_left(1000);
    delay(300); //为在黑线上时先启动走出黑线，摩擦轮电机走出限位开关
    Serial.println("走出黑线delay结束");
    do
    {
      left_patrol();
      Serial.print("D8state");  Serial.println(D8state);
      Serial.print("D9state");  Serial.println(D9state);

      if (D8state == 1 && D9state == 1)
      {
        left_black_line = 1;
        STOP();
      }
      Serial.print("left_black_line=="); Serial.println(left_black_line);
    } while ( left_black_line != 1);

    motor2006_move(-5000);
    delay(300);
    do
    {
      clamp_move_to_B();
    }
    while (grip_now_position != 2);
    Serial.print("开始夹取第"); Serial.print(finish_box_num + 1); Serial.println("个箱子");
    clamp_pick();           //到位后夹取到平台上
    receive_from_pi();      //此处有while接收到信号并判断夹爪目标位置后跳出
    drop_to_car_position(); //有switch /while /grip_now_position到相应值后完成动作后跳出
  }
  Serial.print("finish_box_num");
  Serial.println(finish_box_num);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  顶层型函数
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void start_stage0() //出发右移
{

  Serial.print("stage==");  Serial.println(stage);

  grip_now_position = 0;
  do
  {
    clamp_move_to_A();
  }
  while (grip_now_position != 1);

  right_patrol();
  if (D4state == 1 && D5state == 1)
  {
    delay(20);
    D4state = digitalRead(4);
    D5state = digitalRead(5);
    if (D4state == 1 && D5state == 1)
    {
      STOP();
      do
      {
        clamp_move_to_C();
      }
      while (grip_now_position != 3);
      delay(300);
      stage = 1;

    }
  }
}
void start_stage1() //短距离前进
{
  Serial.print("stage");
  Serial.println(stage);
  grip_now_position = 0;
  do
  {
    clamp_move_to_C();
  }
  while (grip_now_position != 3);
  move_forward(1000);
  D6state = digitalRead(6);
  D7state = digitalRead(7);
  if (D6state == 1 && D7state == 1)
  {
    delay(20);
    D6state = digitalRead(6);
    D7state = digitalRead(7);
    if (D6state == 1 && D7state == 1)
    {
      STOP();
      stage = 2;
      delay(100);
    }
  }
}
void start_stage2() //取物分拣
{
  Serial.print("stage");
  Serial.println(stage);
  if (start_box_1 == 1)
  {
    box_1();   ////有while grip_now_position到相应值后完成动作后跳出
  }
  box_2_6(); //for中嵌套while
  grip_now_position = 0;

  /* do
    {
     clamp_move_to_B();
    }
    while (grip_now_position != 2);*/
  motor2006_move(5000);
  delay(1200);
  motor2006_move(-5000);
  delay(600);
  stage = 3;
}
void start_stage3() //后退
{

  back_patrol();
  if (D6state == 1 && D7state == 1)
  {
    delay(20);
    D6state = digitalRead(6);
    D7state = digitalRead(7);
    if (D6state == 1 && D7state == 1)
    {
      STOP();
      do
      {
        clamp_move_to_A();
      }
      while (grip_now_position != 1);
      stage = 4;
    }
  }
}
void start_stage4() //右移穿过障碍物
{
  clamp_move_to_B(); //取货装置回到中间让重心靠近中间一些
  right_patrol();
  if (D4state == 1 && D5state == 1)
  {
    delay(20);
    D4state = digitalRead(4);
    D5state = digitalRead(5);
    if (D4state == 1 && D5state == 1)
    {
      STOP();
      do
      {
        clamp_move_to_B();
      }
      while (grip_now_position != 2);
      stage = 5;
    }
  }
}
void start_stage5() //后退准备放物
{
  back_patrol();

  if (D6state == 1 && D7state == 1)
  {
    delay(20);
    D6state = digitalRead(6);
    D7state = digitalRead(7);
    if (D6state == 1 && D7state == 1)
    {
      STOP();
      do
      {
        clamp_move_to_C();
      }
      while (grip_now_position != 3);
      stage = 6;
    }
  }
}

void put_box_B()
{
  int left_black_line = 0;
  int back_black_line = 0;
  do
  {
    left_patrol();
    if (D8state == 1 && D8state == 1)
    {
      left_black_line = 1;
      STOP();
    }
  } while (left_black_line != 1);
  move_back(500);
  delay(4000); //后退到放货区
  STOP();
  delay(1000);
  Serial.println("后退到放货区");
  digitalWrite(44, HIGH); //电磁锁打开
  delay(100);
  digitalWrite(44, LOW); //电磁锁不能长时间打开
  move_forward(4000);
  STOP();
  put_box_num = 1;
  // delay(500); //后退到放货区
  /*  do
    {
      move_forward(500);
      if (D6state == 1 && D7state == 1)
      {
        back_black_line = 1;
        STOP();
        put_box_num=1;
      }
    } while (back_black_line != 1);*/
}


void put_box_C()
{
  int left_black_line = 0;
  int back_black_line = 0;
  int black_line_num = 0;
  for (black_line_num = 0; black_line_num < 2; black_line_num++)
  {
    move_left(1000);
    delay(300);
    do
    {
      left_patrol();
      if (D8state == 1 && D8state == 1)
      {
        left_black_line = 1;
        STOP();

      }
    } while (left_black_line != 1);
  }
  STOP();
  move_back(500);
  delay(4000); //后退到放货区
  STOP();
  digitalWrite(44, HIGH); //电磁锁打开
  delay(100);
  digitalWrite(44, LOW); //电磁锁不能长时间打开

  move_forward(500);
  delay(4000);
  STOP();
  do
  {
    clamp_move_to_A();
  }
  while (grip_now_position != 1);
  put_box_num = 2;
}
void put_box_A()
{
  int right_black_line = 0;
  int back_black_line = 0;
  do
  {
    right_patrol();
    if (D4state == 1 && D5state == 1)
    {
      right_black_line = 1;
      STOP();
    }
  } while (right_black_line != 1);
  move_back(500);
  delay(4000); //后退到放货区
  STOP();
  digitalWrite(42, HIGH); //电磁锁打开
  delay(100);
  digitalWrite(42, LOW); //电磁锁不能长时间打开
  move_forward(5000);
  delay(1000);
  STOP();
  while (1);
}

void start_stage6() //放货
{
  switch (put_box_num)
  { case 0: put_box_B(); break;
    case 1: put_box_C(); break;
    case 2: put_box_A(); break;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  方案函数
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void STAR_A()
{
  switch (stage)
  {
    case 0:
      start_stage0();
      break;
    case 1:
      start_stage1();
      break;
    case 2:
      start_stage2();
      break;
    case 3:
      start_stage3();
      break;
    case 4:
      start_stage4();
      break;
    case 5:
      start_stage5();
      break;
    case 6:
      start_stage6();
      break;
    default:
      break;
  }

}
void STAR_B()
{
  switch (stage)
  {
    case 5:
      start_stage5();
      break;
    case 6:
      start_stage6();
      break;
    default:
      break;
  }

}
