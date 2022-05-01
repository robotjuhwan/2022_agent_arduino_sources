#include <Servo.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include "MS5837.h"
#include "hero_msgs/hero_agent_cont.h"
#include "hero_msgs/hero_agent_state.h"

ros::NodeHandle nh;

hero_msgs::hero_agent_state state_msg;
ros::Publisher pub_state("/hero_agent/state", &state_msg);

MS5837 DEPTH_Sensor;

#define RELAY0 15
#define GRIPPER_SIG 20
#define LED_SIG 21

#define RTS 10
Servo GRIPPER_SERVO;

int pwm_m = 1500;

int pwm_m0 = 1500, pid_pwm_m0;
int pwm_m1 = 1500, pid_pwm_m1;
int pwm_m2 = 1500, pid_pwm_m2;
int pwm_m3 = 1500, pid_pwm_m3;
int pwm_m4 = 1500, pid_pwm_m4;
int pwm_m5 = 1500, pid_pwm_m5;

float roll, pitch, yaw;
uint8_t ahrs_valid;
float depth, pressure, temperature;

float yaw_p = 0, yaw_i = 0, yaw_d = 0;
float depth_p = 0, depth_i = 0, depth_d = 0;
float target_yaw = 0, target_depth = 0;

int8_t yaw_con_on = 0, depth_con_on = 0;

float pre_yaw_err = 0;
float yaw_err = 0;
float yaw_err_err = 0;
float yaw_err_sum = 0;

float pre_depth_err = 0;
float depth_err = 0;
float depth_err_err = 0;
float depth_err_sum = 0;

int loop_count = 0;

void messageCont_YAW(const hero_msgs::hero_agent_cont &control_msg)
{
  yaw_p = control_msg.PID_P;
  yaw_i = control_msg.PID_I;
  yaw_d = control_msg.PID_D;

  yaw_con_on = control_msg.Cont_on;

  target_yaw = control_msg.Target_value;
}

void messageCont_DEPTH(const hero_msgs::hero_agent_cont &control_msg)
{
  depth_p = control_msg.PID_P;
  depth_i = control_msg.PID_I;
  depth_d = control_msg.PID_D;

  depth_con_on = control_msg.Cont_on;

  target_depth = control_msg.Target_value;
}

void messageCommand(const std_msgs::Int8 &command_msg)
{
  int Command = command_msg.data;

  if (Command == 0x01)
  {
    digitalWrite(RELAY0, HIGH);
    delay(5000);
  }
  else if (Command == 0x02)
  {
    digitalWrite(RELAY0, LOW);
  }
  else if (Command == 0x03)
  {
    digitalWrite(LED_SIG, HIGH);
  }
  else if (Command == 0x04)
  {
    digitalWrite(LED_SIG, LOW);
  }
  else if (Command == 0x05)
  {
    GRIPPER_SERVO.writeMicroseconds(1800);
  }
  else if (Command == 0x06)
  {
    GRIPPER_SERVO.writeMicroseconds(1400);
  }
  else if (Command == 0x07)
  {
    GRIPPER_SERVO.writeMicroseconds(1200);
  }
  else if (Command == 0x08)
  {
    pwm_m0 = 1500;
    pwm_m1 = 1500;
    pwm_m2 = 1500;
    pwm_m3 = 1500;
    pwm_m4 = 1500;
    pwm_m5 = 1500;
  }
  else if (Command == 9)
  {
    pwm_m0+=10;
  }
  else if (Command == 10)
  {
    pwm_m0-=10;
  }
  else if (Command == 11)
  {
    pwm_m1+=10;
  }
  else if (Command == 12)
  {
    pwm_m1-=10;
  }
  else if (Command == 13)
  {
    pwm_m2+=10;
  }
  else if (Command == 14)
  {
    pwm_m2-=10;
  }
  else if (Command == 15)
  {
    pwm_m3+=10;
  }
  else if (Command == 16)
  {
    pwm_m3-=10;
  }
  else if (Command == 17)
  {
    pwm_m4+=10;
  }
  else if (Command == 18)
  {
    pwm_m4-=10;
  }
  else if (Command == 19)
  {
    pwm_m5+=10;
  }
  else if (Command == 20)
  {
    pwm_m5-=10;
  }
}
ros::Subscriber<hero_msgs::hero_agent_cont> sub_yaw_cont("/hero_agent/yaw_cont",
                                                         &messageCont_YAW);
ros::Subscriber<hero_msgs::hero_agent_cont> sub_depth_cont(
    "/hero_agent/depth_cont", &messageCont_DEPTH);
ros::Subscriber<std_msgs::Int8> sub_command("/hero_agent/command",
                                            &messageCommand);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  Wire.begin();

  pinMode(RELAY0, OUTPUT);
  pinMode(LED_SIG, OUTPUT);

  pinMode(RTS, OUTPUT);
  GRIPPER_SERVO.attach(GRIPPER_SIG);
  while (!DEPTH_Sensor.init())
  {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  DEPTH_Sensor.setModel(MS5837::MS5837_30BA);
  DEPTH_Sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  digitalWrite(RTS, HIGH);

  nh.initNode();
  nh.advertise(pub_state);
  nh.subscribe(sub_yaw_cont);
  nh.subscribe(sub_depth_cont);
  nh.subscribe(sub_command);
}

void loop()
{
  union {
    float real;
    uint32_t base;
  } u_real_base;

  int offset = 0;

  byte ahrs_buffer[32];
  ahrs_input();
  Serial1.readBytes(ahrs_buffer, 32);
  /*
for(int i=0;i<32;i++)
{
  Serial.print(ahrs_buffer[i]);
  Serial.print(' ');
}
Serial.println(' ');
*/
  u_real_base.base = 0;
  u_real_base.base |= ((uint32_t)(ahrs_buffer[9])) << (8 * 0);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[8])) << (8 * 1);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[7])) << (8 * 2);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[6])) << (8 * 3);
  roll = u_real_base.real;

  u_real_base.base = 0;
  u_real_base.base |= ((uint32_t)(ahrs_buffer[13])) << (8 * 0);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[12])) << (8 * 1);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[11])) << (8 * 2);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[10])) << (8 * 3);
  pitch = u_real_base.real;

  u_real_base.base = 0;
  u_real_base.base |= ((uint32_t)(ahrs_buffer[17])) << (8 * 0);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[16])) << (8 * 1);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[15])) << (8 * 2);
  u_real_base.base |= ((uint32_t)(ahrs_buffer[14])) << (8 * 3);
  yaw = u_real_base.real;
  /*
  roll = ahrs_buffer[6] << 24 | ahrs_buffer[7] << 16 | ahrs_buffer[8] << 6 |
  ahrs_buffer[9]; pitch = ahrs_buffer[10] << 24 | ahrs_buffer[11] << 16 |
  ahrs_buffer[12] << 6 | ahrs_buffer[13]; yaw = ahrs_buffer[14] << 24 |
  ahrs_buffer[15] << 16 | ahrs_buffer[16] << 6 | ahrs_buffer[17];
*/
  ahrs_valid = ahrs_buffer[18] * 256 + ahrs_buffer[19];

  /*
  Serial.println("dd");
  Serial.println(roll);
  Serial.println(pitch);
  Serial.println(yaw);
  Serial.println(ahrs_valid);
*/


  loop_count++;
  if(loop_count==20)
  {
    DEPTH_Sensor.read1();
  }
  else if(loop_count==50)
  {
    DEPTH_Sensor.read2();
  }
  else if(loop_count==80)
  {
    DEPTH_Sensor.read3();
  }
  else if (loop_count > 100)
  {
    loop_count = 0;
  

  depth = DEPTH_Sensor.depth();
  pressure = DEPTH_Sensor.pressure();
  temperature = DEPTH_Sensor.temperature();
  
    state_msg.Depth = depth;
    state_msg.Roll = roll;
    state_msg.Pitch = pitch;
    state_msg.Yaw = yaw;
    state_msg.Pressure = pressure;
    state_msg.Temperature = temperature;
    pub_state.publish(&state_msg);
  }

  if (yaw_con_on==1)
  {
    pre_yaw_err = yaw_err;
    yaw_err = target_yaw - yaw;

    yaw_err_err = yaw_err - pre_yaw_err;

    yaw_err_sum = yaw_err_sum + yaw_err;

    pid_pwm_m0 = pid_pwm_m0 + yaw_p * yaw_err + yaw_i * yaw_err_sum +
                 yaw_d * yaw_err_err;

    if (pid_pwm_m0 > 100)
      pid_pwm_m0 = 100;
    else if (pid_pwm_m0 < -100)
      pid_pwm_m0 = -100;
    pwm_m1 = 1600 + pid_pwm_m0;
    pwm_m2 = 1400 + pid_pwm_m0;
    pwm_m4 = 1400 - pid_pwm_m0;
    pwm_m5 = 1600 - pid_pwm_m0;
  }
  else if(yaw_con_on==2)
  {
    pwm_m1 = 1500;
    pwm_m2 = 1500;
    pwm_m4 = 1500;
    pwm_m5 = 1500;
  }

  if (depth_con_on==1)
  {
    pre_depth_err = depth_err;
    depth_err = depth - target_depth;

    depth_err_err = depth_err - pre_depth_err;

    depth_err_sum = depth_err_sum + depth_err;

    pid_pwm_m1 = pid_pwm_m1 + depth_p * depth_err + depth_i * depth_err_sum +
                 depth_d * depth_err_err;

    if (pid_pwm_m1 > 100)
      pid_pwm_m1 = 100;
    else if (pid_pwm_m1 < -100)
      pid_pwm_m1 = -100;

    pwm_m0 = 1500 + pid_pwm_m1;
    pwm_m3 = 1500 + pid_pwm_m1;
  }
  else if(depth_con_on==2)
  {
    pwm_m0 = 1500;
    pwm_m3 = 1500;
  }

  esc_input(0x01, pwm_m0, pwm_m1, pwm_m2);
  esc_input(0x02, pwm_m3, pwm_m4, pwm_m5);
  nh.spinOnce();

  //delay(1);
  /*
  if (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == 'q')
    {
    }
    else if (inChar == 'w')
    {
    }
    else if (inChar == 'a')
    {
      digitalWrite(RELAY0, HIGH);
    }
    else if (inChar == 's')
    {
      digitalWrite(RELAY0, LOW);
    }
    else if (inChar == 'd')
    {
      digitalWrite(LED_SIG, HIGH);
    }
    else if (inChar == 'f')
    {
      digitalWrite(LED_SIG, LOW);
    }
    else if (inChar == 'g')
    {
      pwm_m = 1800;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 'h')
    {
      pwm_m = 1400;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 'j')
    {
      pwm_m = 1200;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 'z')
    {
      pwm_m0 = 1500;
      pwm_m1 = 1500;
      pwm_m2 = 1500;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");

      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'x')
    {
      pwm_m0 += 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'c')
    {
      pwm_m0 -= 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'v')
    {
      pwm_m1 += 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'b')
    {
      pwm_m1 -= 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'n')
    {
      pwm_m2 += 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    else if (inChar == 'm')
    {
      pwm_m2 -= 10;
      Serial.println(pwm_m0);
      Serial.println(pwm_m1);
      Serial.println(pwm_m2);
      Serial.println(" ");
      esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    }
    esc_input(0x01, pwm_m0, pwm_m1, pwm_m2);
  }
  */
  // put your main code here, to run repeatedly:
}

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2)
{
  uint8_t buf0[10] = {0xff, 0xff, ID, pwm0 / 256,
                      pwm0 % 256, pwm1 / 256, pwm1 % 256, pwm2 / 256,
                      pwm2 % 256, 0xfe};
  Serial2.write(buf0, 10);
}

void ahrs_input(void)
{
  Serial1.write(0x75);
  Serial1.write(0x65);
  Serial1.write(0x0c);
  Serial1.write(0x04);
  Serial1.write(0x04);
  Serial1.write(0x03);
  Serial1.write(0x00);
  Serial1.write(0x00);
  Serial1.write(0xf1);
  Serial1.write(0xe0);
}
