#include <avr/interrupt.h>

#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "MS5837.h"

#include <ros.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "hero_msgs/hero_agent_state.h"
#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"
#include "hero_msgs/hero_xy_cont.h"
#include "hero_msgs/hero_command.h"

// for PIN SETTING---------------------
#define F_CPU 16000000UL

#define RELAY 15
#define GRIPPER_SIG 20
#define LED_SIG 21

#define RTS 10
//--------------------------------------

// for ROS serial libaray for PUBLISH---------------------
ros::NodeHandle nh; // main handle

hero_msgs::hero_agent_state state_msg;    // state
hero_msgs::hero_agent_sensor sensors_msg; // sensors

ros::Publisher pub_state("/hero_agent/state", &state_msg);
ros::Publisher pub_sensors("/hero_agent/sensors", &sensors_msg);
//--------------------------------------

// for DEPTH sensor---------------------

MS5837 DEPTH_Sensor;

double depth;
//--------------------------------------

// for AHRS_3DM_GX5----------------------
volatile int serial_count = 0;
volatile uint8_t inputString[100];       // a String to hold incoming data
volatile boolean stringComplete = false; // whether the string is complete

volatile uint8_t imu_check = 0;

volatile union
{
  float real;
  uint32_t base;
} u_data;

volatile float roll, pitch, yaw;
volatile float yaw_temp;
volatile float acc_roll, acc_pitch, acc_yaw;
volatile float acc_x = 0, acc_y = 0, acc_z = 0;
volatile uint8_t ahrs_valid1, ahrs_valid2, ahrs_valid3;

volatile uint8_t yaw_calid_command = 0;
volatile char yaw_calib = 0;
volatile float pre_yaw = 0;
volatile float yaw_calib2 = 0;
//--------------------------------------

// for Trusters-------------------------
volatile int pwm_m0 = 1500, pid_pwm_m0;
volatile int pwm_m1 = 1500, pid_pwm_m1;
volatile int pwm_m2 = 1500, pid_pwm_m2;
volatile int pwm_m3 = 1500, pid_pwm_m3;
volatile int pwm_m4 = 1500, pid_pwm_m4;
volatile int pwm_m5 = 1500, pid_pwm_m5;
//--------------------------------------

// for Control yaw and depth---------------------
volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;

volatile double T = 0.004;      // Loop time.
volatile double T_depth = 0.04; // Loop time.

volatile int throttle = 40;

volatile double P_angle_gain_yaw = 6.5;
volatile double P_gain_yaw = 10;
volatile double I_gain_yaw = 1;
volatile double D_gain_yaw = 0.5;

double P_gain_depth = 1000.0;
double I_gain_depth = 10.0;
double D_gain_depth = 100.0;

volatile double error_yaw;
volatile double error_pid_yaw, error_pid_yaw1;
volatile double P_angle_pid_yaw;
volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
volatile double desired_angle_yaw = 1;

double error_pid_depth, error_pid_depth1;
double P_angle_pid_depth;
double P_depth, I_depth, D_depth, PID_depth;
double desired_angle_depth = 0.5;
//--------------------------------------

// for Control xy position---
volatile uint8_t cont_direc = 0;
volatile int move_speed = 10;
volatile int added_move_speed = 0;

volatile float object_x = 0, object_y = 0;
volatile uint8_t object_valid = 0;
volatile double costheta = 0;
volatile double sintheta = 0;
//------------------------------------------

// for RECOVERY movement------------------------
volatile int cont_recovery = 0;

float ipcam_qr_x = 0;
float ipcam_qr_y = 0;
float ipcam_qr_z = 0;
float ipcam_qr_yaw = 0;
int ipcam_qr_valid = 0;

int yaw_stage = 0;
//-------------------------------------

// for STATE addition
volatile uint8_t cont_Yaw = 0;
volatile uint8_t cont_Depth = 0;
volatile uint8_t state_Relay = 0;
volatile uint8_t state_Laser = 0;

volatile int State_all = 0;
//-------------------------------------

void messageCont_IPCAM(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg &ipcam_msg)
{
  ipcam_qr_x = ipcam_msg.T_X;
  ipcam_qr_y = ipcam_msg.T_Y;
  ipcam_qr_z = ipcam_msg.T_Z;
  ipcam_qr_yaw = ipcam_msg.T_YAW;
  ipcam_qr_valid = ipcam_msg.T_valid;
}

void messageCont_XY(const hero_msgs::hero_xy_cont &hero_xy_msg)
{
  object_x = hero_xy_msg.TARGET_X;
  object_y = hero_xy_msg.TARGET_Y;
  object_valid = hero_xy_msg.VALID;
}

void hero_command_result(const hero_msgs::hero_command::Request &req,
                         hero_msgs::hero_command::Response &res)
{
  // The service name is 'ros_tutorial_srv' and it will call 'calculation' function upon the service request.
  int Command = req.command;

  if (Command == 'n')
  {
    desired_angle_yaw = 0;
    yaw_calid_command = 1;
  }
  else if (Command == 'q')
  {
    cont_direc = 0;
  }
  else if (Command == 's') // backward
  {
    cont_direc = 1;
  }
  else if (Command == 'w') // forward
  {
    cont_direc = 2;
  }
  else if (Command == 'd') // right
  {
    cont_direc = 3;
  }
  else if (Command == 'a') // left
  {
    cont_direc = 4;
  }
  else if (Command == '1') // concon
  {
    cont_direc = 5;
  }
  else if (Command == '2') // concon
  {
    cont_recovery = 1;
  }
  else if (Command == '3') // concon
  {
    cont_recovery = 0;
    yaw_stage = 0;
  }
  else if (Command == 'z') // left
  {
    move_speed += 10;
  }
  else if (Command == 'x') // right
  {
    move_speed -= 10;
  }
  else if (Command == 'e')
  {
    digitalWrite(RELAY, HIGH);
    delay(5000);
  }
  else if (Command == 't')
  {
    digitalWrite(RELAY, LOW);
  }
  else if (Command == 'r')
  {
    digitalWrite(LED_SIG, HIGH);
  }
  else if (Command == 'f')
  {
    digitalWrite(LED_SIG, LOW);
  }
  else if (Command == 'c')
  {
    OCR1A = 450; // open gripper
  }
  else if (Command == 'v')
  {
    OCR1A = 350; // stop gripper
  }
  else if (Command == 'b')
  {
    OCR1A = 300; // close gripper
  }
  else if (Command == 'g')
  {

    pwm_m0 = 1500;
    pwm_m1 = 1500;
    pwm_m2 = 1500;
    pwm_m3 = 1500;
    pwm_m4 = 1500;
    pwm_m5 = 1500;

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }
  else if (Command == 'y')
  {
    cont_yaw_on = 1;
  }
  else if (Command == 'h')
  {
    cont_yaw_on = 0;
  }
  else if (Command == 'u')
  {
    throttle += 10;
  }
  else if (Command == 'j')
  {
    throttle -= 10;
  }
  else if (Command == 'i')
  {
    desired_angle_yaw += 0.1;
  }
  else if (Command == 'k')
  {
    desired_angle_yaw -= 0.1;
  }
  else if (Command == '6')
  {
    desired_angle_yaw += 0.01;
  }
  else if (Command == '5')
  {
    desired_angle_yaw -= 0.01;
  }
  else if (Command == 'o')
  {
    desired_angle_depth += 0.1;
  }
  else if (Command == 'l')
  {
    desired_angle_depth -= 0.1;
  }
  else if (Command == '.')
  {
    desired_angle_depth += 0.01;
  }
  else if (Command == ',')
  {
    desired_angle_depth -= 0.01;
  }
  else if (Command == 'p')
  {
    cont_depth_on = 1;
  }
  else if (Command == ';')
  {
    cont_depth_on = 0;
  }

  res.result = 1;
}

void messageCommand(const std_msgs::Int8 &command_msg)
{
  int Command = command_msg.data;

  if (Command == 'n')
  {
    desired_angle_yaw = 0;
    yaw_calid_command = 1;
  }
  else if (Command == 'q')
  {
    cont_direc = 0;
  }
  else if (Command == 's') // backward
  {
    cont_direc = 1;
  }
  else if (Command == 'w') // forward
  {
    cont_direc = 2;
  }
  else if (Command == 'd') // right
  {
    cont_direc = 3;
  }
  else if (Command == 'a') // left
  {
    cont_direc = 4;
  }
  else if (Command == '1') // concon
  {
    cont_direc = 5;
  }
  else if (Command == '2') // concon
  {
    cont_recovery = 1;
  }
  else if (Command == '3') // concon
  {
    cont_recovery = 0;
    yaw_stage = 0;
  }
  else if (Command == 'z') // left
  {
    move_speed += 10;
  }
  else if (Command == 'x') // right
  {
    move_speed -= 10;
  }
  else if (Command == 'e')
  {
    state_Relay = 1;
    digitalWrite(RELAY, HIGH);
    delay(5000);
  }
  else if (Command == 't')
  {
    state_Relay = 0;
    digitalWrite(RELAY, LOW);
  }
  else if (Command == 'r')
  {
    state_Laser = 1;
    digitalWrite(LED_SIG, HIGH);
  }
  else if (Command == 'f')
  {
    state_Laser = 0;
    digitalWrite(LED_SIG, LOW);
  }
  else if (Command == 'c')
  {
    OCR1A = 450; // open gripper
  }
  else if (Command == 'v')
  {
    OCR1A = 350; // stop gripper
  }
  else if (Command == 'b')
  {
    OCR1A = 300; // close gripper
  }
  else if (Command == 'g')
  {

    pwm_m0 = 1500;
    pwm_m1 = 1500;
    pwm_m2 = 1500;
    pwm_m3 = 1500;
    pwm_m4 = 1500;
    pwm_m5 = 1500;

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }
  else if (Command == 'y')
  {
    cont_yaw_on = 1;
    cont_Yaw = 1;
  }
  else if (Command == 'h')
  {
    cont_yaw_on = 0;
    cont_Yaw = 0;
  }
  else if (Command == 'u')
  {
    throttle += 10;
  }
  else if (Command == 'j')
  {
    throttle -= 10;
  }
  else if (Command == 'i')
  {
    desired_angle_yaw += 0.1;
  }
  else if (Command == 'k')
  {
    desired_angle_yaw -= 0.1;
  }
  else if (Command == '6')
  {
    desired_angle_yaw += 0.01;
  }
  else if (Command == '5')
  {
    desired_angle_yaw -= 0.01;
  }
  else if (Command == 'o')
  {
    desired_angle_depth += 0.1;
  }
  else if (Command == 'l')
  {
    desired_angle_depth -= 0.1;
  }
  else if (Command == '.')
  {
    desired_angle_depth += 0.01;
  }
  else if (Command == ',')
  {
    desired_angle_depth -= 0.01;
  }
  else if (Command == 'p')
  {
    cont_depth_on = 1;
    cont_Depth = 1;
  }
  else if (Command == ';')
  {
    cont_depth_on = 0;
    cont_Depth = 0;
  }
  State_all = 0;
  if (cont_Yaw == 1)
  {
    State_all += 1;
  }
  if (cont_Depth == 1)
  {
    State_all += 2;
  }
  if (state_Relay == 1)
  {
    State_all += 4;
  }
  if (state_Laser == 1)
  {
    State_all += 8;
  }
  if (cont_recovery == 1)
  {
    State_all += 16;
  }
}

ros::Subscriber<std_msgs::Int8> sub_command("/hero_agent/command",
                                            &messageCommand);

ros::Subscriber<hero_msgs::hero_xy_cont> sub_xy_msg(
    "/hero_agent/xy", &messageCont_XY);

ros::Subscriber<ros_opencv_ipcam_qr::hero_ipcam_qr_msg> sub_ipcom_qr_msg(
    "/hero_ipcam_qr_msg", &messageCont_IPCAM);

ros::ServiceServer<hero_msgs::hero_command::Request, hero_msgs::hero_command::Response> ros_command_srv_ser("agent_command_srv", &hero_command_result);

//--------------------------------------------
volatile int check_hz = 0;
// Incoming AHRS data--------------------------------------
ISR(USART1_RX_vect)
{ // USART1에 RX가 들어왔을때 동작하는 interrupt

  char inChar = (char)UDR1;

  if (imu_check == 0 && inChar == (char)0x75)
  {
    imu_check = 1;
    inputString[0] = inChar;
  }
  else if (imu_check == 1 && inChar == (char)0x65)
  {
    imu_check = 2;
    inputString[1] = inChar;
  }
  else if (imu_check == 2 && inChar == (char)0x82)
  {
    imu_check = 3;
    inputString[2] = inChar;
    serial_count = 3;
  }
  else if (imu_check == 3)
  {
    inputString[serial_count] = inChar;
    serial_count++;
  }
  // if the incoming character is a newline, set a flag so the main loop can
  // do something about it:
  // inputString[19] == 1 &&
  if (serial_count == 48 && inputString[35] == 1) //수신이 완료
  {
    stringComplete = true;
    serial_count = 0;
    imu_check = 0;
    check_hz++;
    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[9]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[8]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[7]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[6]) << (8 * 3);
    roll = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[13]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[12]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[11]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[10]) << (8 * 3);
    pitch = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[17]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[16]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[15]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[14]) << (8 * 3);
    yaw = u_data.real;
    yaw_temp = u_data.real;
    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[25]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[24]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[23]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[22]) << (8 * 3);
    acc_roll = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[29]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[28]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[27]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[26]) << (8 * 3);
    acc_pitch = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[33]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[32]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[31]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[30]) << (8 * 3);
    acc_yaw = u_data.real;
    /*
    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[41]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[40]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[39]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[38]) << (8 * 3);
    if(u_data.real>-0.01&&u_data.real<0.01)
    acc_x += u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[45]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[44]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[43]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[42]) << (8 * 3);
    if(u_data.real>-0.01&&u_data.real<0.01)
    acc_y += u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[49]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[48]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[47]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[46]) << (8 * 3);
    if(u_data.real>-0.01&&u_data.real<0.01)
    acc_z += u_data.real;
*/
    ahrs_valid1 = inputString[19];
    ahrs_valid2 = inputString[35];
    ahrs_valid3 = inputString[51];

    yaw -= yaw_calib2;

    if (yaw_calid_command == 1)
    {
      yaw_calid_command = 0;
      yaw_calib2 = yaw_temp;
      pre_yaw = 0;
      yaw = 0;
      yaw_calib = 0;
    }

    if (yaw - pre_yaw > 4.712388)
      yaw_calib--;
    else if (yaw - pre_yaw < -4.712388)
      yaw_calib++;
    pre_yaw = yaw;
    yaw += 3.141592 * yaw_calib * 2;

    //
    // yaw control loop

    // esc_input(0x01, pwm_m0, pwm_m1, pwm_m2);
    // esc_input(0x02, pwm_m3, pwm_m4, pwm_m5);
  }
  else if (serial_count > 48) //데이터가 10개 넘게 들어옴 (데inputString[19]이터 수신 실패)
  {
    inputString[0] = '\0';
    serial_count = 0;
    stringComplete = false;
    imu_check = 0;
  }
}

void UART1_write(char ch)
{

  while (!(UCSR1A & 0x20))
    ;

  UDR1 = ch;
}
void UART2_write(char ch)
{

  while (!(UCSR2A & 0x20))
    ;

  UDR2 = ch;
}
//----------------------------------------------------------

void setup()
{
  nh.initNode();

  nh.advertise(pub_state);
  nh.advertise(pub_sensors);
  nh.subscribe(sub_xy_msg);
  nh.subscribe(sub_command);
  nh.subscribe(sub_ipcom_qr_msg);

  nh.advertiseService(ros_command_srv_ser);

  Initialization(); // init all 1100ms
}
// loop is for control
void loop()
{
  if (cont_yaw_on == 1)
    PID_control_yaw();
  nh.spinOnce();
  DEPTH_Sensor.read();
  depth = DEPTH_Sensor.depth();

  if (cont_depth_on == 1)
    PID_control_depth();

  
    sensors_msg.ROLL = roll;
  sensors_msg.PITCH = pitch;
  sensors_msg.YAW = yaw;
  sensors_msg.DEPTH = depth;

  pub_sensors.publish(&sensors_msg);
  /*


    state_msg.Yaw = yaw;
    state_msg.Target_yaw = desired_angle_yaw;
    state_msg.Throttle = throttle;
    state_msg.Valid_yaw = inputString[19];
    state_msg.Depth = depth;
    state_msg.Target_depth = desired_angle_depth;
    state_msg.Move_speed = move_speed;

    pub_state.publish(&state_msg);


    state_msg.Yaw = yaw;
    state_msg.Target_yaw = desired_angle_yaw;
    state_msg.Throttle = throttle;
    state_msg.Valid_yaw = inputString[19];
    state_msg.Depth = depth;
    state_msg.Target_depth = desired_angle_depth;
    state_msg.Move_speed = move_speed;

    pub_state.publish(&state_msg);
     */

  state_msg.Yaw = yaw;
  state_msg.Target_yaw = desired_angle_yaw;
  state_msg.Throttle = throttle;
  state_msg.Valid_yaw = inputString[19];
  state_msg.Depth = depth;
  state_msg.Target_depth = desired_angle_depth;
  state_msg.Move_speed = move_speed;
  state_msg.Cont_state = cont_direc;
  state_msg.State_addit = State_all;
  pub_state.publish(&state_msg);

  if (cont_recovery == 1)
  {
    control_recovery();
  }
}

void control_recovery(void)
{
  if (ipcam_qr_valid == 1)
  {
    if (ipcam_qr_z > 0.5)
    {
      added_move_speed = 10;
      yaw_stage = 0;
      move_speed = 10;
      if (ipcam_qr_x > 0.01)
      {
        cont_direc = 3;
        // left
      }
      else if (ipcam_qr_x < -0.01)
      {
        cont_direc = 4;
        // right
      }
      else
      {
        cont_direc = 2;
        // forward
        if (ipcam_qr_yaw > 0.01)
        {
          desired_angle_yaw -= 0.005;
        }
        else if (ipcam_qr_yaw < -0.01)
        {
          desired_angle_yaw += 0.005;
        }
      }

      if (ipcam_qr_y > -0.04)
      {
        desired_angle_depth -= 0.001;
      }
      else if (ipcam_qr_y < -0.06)
      {
        desired_angle_depth += 0.001;
      }
    }
    else if (yaw_stage == 0) //<0.5
    {
      added_move_speed = 10;
      move_speed = 20;
      if (ipcam_qr_x > 0.01)
      {
        cont_direc = 3;
        // left
      }
      else if (ipcam_qr_x < -0.01)
      {
        cont_direc = 4;
        // right
      }
      else
      {
        cont_direc = 0;
        // forward
        if (ipcam_qr_yaw > 0.01)
        {
          desired_angle_yaw -= 0.001;
        }
        else if (ipcam_qr_yaw < -0.01)
        {
          desired_angle_yaw += 0.001;
        }
        else
        {
          yaw_stage = 1;
        }
      }

      if (ipcam_qr_y > -0.04)
      {
        desired_angle_depth -= 0.001;
      }
      else if (ipcam_qr_y < -0.06)
      {
        desired_angle_depth += 0.001;
      }
    }
    else if (yaw_stage == 1) //<0.5
    {
      added_move_speed = 10;
      if (ipcam_qr_x > 0.001)
      {
        move_speed = 10;
        cont_direc = 3;
        // left
      }
      else if (ipcam_qr_x < -0.001)
      {
        move_speed = 10;
        cont_direc = 4;
        // right
      }
      else
      {
        move_speed = 30;
        cont_direc = 2;
        // forward
        if (ipcam_qr_yaw > 0.1)
        {
          desired_angle_yaw -= 0.001;
        }
        else if (ipcam_qr_yaw < -0.1)
        {
          desired_angle_yaw += 0.001;
        }
      }
      if (ipcam_qr_z >= 0.2)
      {
        if (ipcam_qr_y > -0.03)
        {
          desired_angle_depth -= 0.001;
        }
        else if (ipcam_qr_y < -0.05)
        {
          desired_angle_depth += 0.001;
        }

        if (ipcam_qr_yaw > 0.1)
        {
          desired_angle_yaw -= 0.001;
        }
        else if (ipcam_qr_yaw < -0.1)
        {
          desired_angle_yaw += 0.001;
        }
      }

      else if (ipcam_qr_z < 0.17)
      {
        added_move_speed = 0;
        move_speed = 40;
        cont_direc = 2;
        if (ipcam_qr_y > 0.00)
        {
          desired_angle_depth -= 0.001;
        }
        else if (ipcam_qr_y < -0.02)
        {
          desired_angle_depth += 0.001;
        }

        if (ipcam_qr_yaw > 0.1)
        {
          desired_angle_yaw -= 0.001;
        }
        else if (ipcam_qr_yaw < -0.1)
        {
          desired_angle_yaw += 0.001;
        }

        if (ipcam_qr_z < 0.1)
        {
          move_speed = 30;
          added_move_speed = 0;

          cont_recovery = 0;
          yaw_stage = 0;
          cont_direc = 0;
        }
      }
      else if (ipcam_qr_z < 0.2)
      {
        added_move_speed = 10;
        if (ipcam_qr_y > -0.02)
        {
          desired_angle_depth -= 0.001;
        }
        else if (ipcam_qr_y < -0.04)
        {
          desired_angle_depth += 0.001;
        }
      }
    }
  }
}

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2)
{
  UART2_write(0xff);
  UART2_write(0xff);
  UART2_write(ID);
  UART2_write(pwm0 / 256);
  UART2_write(pwm0 % 256);
  UART2_write(pwm1 / 256);
  UART2_write(pwm1 % 256);
  UART2_write(pwm2 / 256);
  UART2_write(pwm2 % 256);
  UART2_write(0xfe);
}

void Initialization(void)
{
  // Serial setting
  // Serial2.begin(115200); //truster control
  UBRR2H = 0x00;
  UBRR2L = 0x08;
  UCSR2A = 0x00;
  UCSR2B = 0x18;
  UCSR2C = 0x06;
  // Serial1 setting (ahrs)
  UBRR1H = 0x00;
  UBRR1L = 0x08;
  UCSR1A = 0x00;
  UCSR1B = 0x98;
  UCSR1C = 0x06;

  // init pressure sensor
  Wire.begin();

  // Init AHRS

  delay(100);
  UART1_write(0x75);
  UART1_write(0x65);
  UART1_write(0x0C);
  UART1_write(0x05);
  UART1_write(0x05);
  UART1_write(0x11);
  UART1_write(0x01);
  UART1_write(0x03);
  UART1_write(0x01);
  UART1_write(0x06);
  UART1_write(0x1E);
  delay(1000);

  // init gripper by servo
  // GRIPPER_SERVO.attach(GRIPPER_SIG);
  DDRB |= 0x20;
  TCCR1A = 0x82;
  TCCR1B = 0x1B;
  TCCR1C = 0;
  ICR1 = 4999;

  OCR1A = 350;

  // depth sensor init
  while (!DEPTH_Sensor.init())
  {
    delay(5000);
  }
  DEPTH_Sensor.setModel(MS5837::MS5837_30BA);
  DEPTH_Sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // basic io init
  pinMode(RELAY, OUTPUT);
  pinMode(LED_SIG, OUTPUT);
  pinMode(RTS, OUTPUT);

  // RTS on (rs485 send)
  digitalWrite(RTS, HIGH);
}

void PID_control_yaw()
{
  error_yaw = desired_angle_yaw - yaw;            // angle def
  P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; // angle def + outer P control

  error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

  P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
  D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
  I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
  I_yaw = constrain(I_yaw, -100, 100);                       // I control must be limited to prevent being jerk.

  PID_yaw = P_yaw + D_yaw + I_yaw;

  if (cont_direc == 0) // stop
  {
    pwm_m1 = -PID_yaw - throttle + 1500;
    pwm_m2 = -PID_yaw + throttle + 1500;
    pwm_m4 = -PID_yaw - throttle + 1500;
    pwm_m5 = -PID_yaw + throttle + 1500;
  }
  else if (cont_direc == 1) // backward
  {
    pwm_m1 = -PID_yaw - throttle + 1500 + move_speed;
    pwm_m2 = -PID_yaw + throttle + 1500 + move_speed;
    pwm_m4 = -PID_yaw - throttle + 1500 - move_speed;
    pwm_m5 = -PID_yaw + throttle + 1500 - move_speed;
  }
  else if (cont_direc == 2) // forward
  {
    pwm_m1 = -PID_yaw - throttle + 1500 - move_speed;
    pwm_m2 = -PID_yaw + throttle + 1500 - move_speed;
    pwm_m4 = -PID_yaw - throttle + 1500 + move_speed;
    pwm_m5 = -PID_yaw + throttle + 1500 + move_speed;
  }
  else if (cont_direc == 3) // right
  {
    pwm_m1 = -PID_yaw - throttle + 1500 + move_speed - added_move_speed;
    pwm_m2 = -PID_yaw + throttle + 1500 - move_speed - added_move_speed;
    pwm_m4 = -PID_yaw - throttle + 1500 - move_speed + added_move_speed;
    pwm_m5 = -PID_yaw + throttle + 1500 + move_speed + added_move_speed;
  }
  else if (cont_direc == 4) // left
  {
    pwm_m1 = -PID_yaw - throttle + 1500 - move_speed - added_move_speed;
    pwm_m2 = -PID_yaw + throttle + 1500 + move_speed - added_move_speed;
    pwm_m4 = -PID_yaw - throttle + 1500 + move_speed + added_move_speed;
    pwm_m5 = -PID_yaw + throttle + 1500 - move_speed + added_move_speed;
  }
  else if (cont_direc == 5) // concon
  {
    // costheta = (object_x-320)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));
    // sintheta = (240-object_y)/sqrt((object_x-320)*(object_x-320)+(object_y-240)*(object_y-240));

    if (object_valid == 0)
    {
      pwm_m1 = -PID_yaw - throttle + 1500;
      pwm_m2 = -PID_yaw + throttle + 1500;
      pwm_m4 = -PID_yaw - throttle + 1500;
      pwm_m5 = -PID_yaw + throttle + 1500;
    }
    else
    {
      costheta = (object_x - 320) / 300;
      sintheta = (240 - object_y) / 300;

      pwm_m1 = -PID_yaw - throttle + 1500 + (float)move_speed * (costheta - sintheta);
      pwm_m2 = -PID_yaw + throttle + 1500 + (float)move_speed * (-costheta - sintheta);
      pwm_m4 = -PID_yaw - throttle + 1500 + (float)move_speed * (-costheta + sintheta);
      pwm_m5 = -PID_yaw + throttle + 1500 + (float)move_speed * (costheta + sintheta);
    }
  }

  // pwm_m1 = constrain(pwm_m1, 1100, 1500);
  // pwm_m2 = constrain(pwm_m2, 1500, 1900);
  // pwm_m4 = constrain(pwm_m4, 1100, 1500);
  // pwm_m5 = constrain(pwm_m5, 1500, 1900);

  pwm_m1 = constrain(pwm_m1, 1200, 1800);
  pwm_m2 = constrain(pwm_m2, 1200, 1800);
  pwm_m4 = constrain(pwm_m4, 1200, 1800);
  pwm_m5 = constrain(pwm_m5, 1200, 1800);

  esc_input(0x02, pwm_m0, 1500, pwm_m2);
  esc_input(0x03, pwm_m3, pwm_m4, 1500);

  error_pid_yaw1 = error_pid_yaw;
}

void PID_control_depth()
{
  error_pid_depth = desired_angle_depth - depth;
  P_depth = error_pid_depth * P_gain_depth;                                // Inner P control
  D_depth = (error_pid_depth - error_pid_depth1) / T_depth * D_gain_depth; // Inner D control
  I_depth += (error_pid_depth)*T_depth * I_gain_depth;                     // Inner I control
  I_depth = constrain(I_depth, -100, 100);                                 // I control must be limited to prevent being jerk.

  PID_depth = P_depth + D_depth + I_depth;

  pwm_m0 = -PID_depth + 1500 - 30;
  pwm_m3 = -PID_depth + 1500 - 30;

  pwm_m0 = constrain(pwm_m0, 1350, 1650);
  pwm_m3 = constrain(pwm_m3, 1350, 1650);

  // esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
  // esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

  error_pid_depth1 = error_pid_depth;
}
