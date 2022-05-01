#include <avr/interrupt.h>
#include <Servo.h>

#define F_CPU 16000000UL

#define RELAY0 15
#define GRIPPER_SIG 20
#define LED_SIG 21

#define RTS 10
Servo GRIPPER_SERVO;

int pwm_m = 1500;

volatile int serial_count = 0;
volatile uint8_t inputString[50];           // a String to hold incoming data
volatile boolean stringComplete = false; // whether the string is complete

volatile uint8_t imu_check = 0;

volatile float aa,bb,cc,dd,ee,ff;

volatile union {
        float real;
        uint32_t base;
      } u_data;


volatile int pwm_m0 = 1500, pid_pwm_m0;
volatile int pwm_m1 = 1500, pid_pwm_m1;
volatile int pwm_m2 = 1500, pid_pwm_m2;
volatile int pwm_m3 = 1500, pid_pwm_m3;
volatile int pwm_m4 = 1500, pid_pwm_m4;
volatile int pwm_m5 = 1500, pid_pwm_m5;

volatile float yaw;

volatile char yaw_calib = 0;
volatile float pre_yaw = 0;
volatile float yaw_calib2 = 0;

volatile int incount = 0;
ISR(USART1_RX_vect)
{ // USART1에 RX가 들어왔을때 동작하는 interrupt

  char inChar = (char)UDR1;

  if(imu_check==0&&inChar==(char)0x75)
  {
    imu_check = 1;
    inputString[0] = inChar;
  }
  else if(imu_check==1&&inChar==(char)0x65)
  {
    imu_check = 2;
    inputString[1] = inChar;
  }
  else if(imu_check==2&&inChar==(char)0x82)
  {
    imu_check = 3;
    inputString[2] = inChar;
    serial_count = 3;
  }
  else if(imu_check==3)
  {
    inputString[serial_count] = inChar;
    serial_count++;
  }
  // if the incoming character is a newline, set a flag so the main loop can
  // do something about it:
  if (serial_count == 38&&inputString[19]==1&&inputString[35]==1) //수신이 완료
  {
    stringComplete = true;
    serial_count = 0;
    imu_check = 0;

     u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[9]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[8]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[7]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[6]) << (8 * 3);
      aa = u_data.real;

      u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[13]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[12]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[11]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[10]) << (8 * 3);
      bb = u_data.real;

      u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[17]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[16]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[15]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[14]) << (8 * 3);
      cc = u_data.real;

      u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[25]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[24]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[23]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[22]) << (8 * 3);
      dd = u_data.real;

      u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[29]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[28]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[27]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[26]) << (8 * 3);
      ee = u_data.real;

      u_data.base = 0;
      u_data.base |= ((uint32_t) inputString[33]) << (8 * 0);
      u_data.base |= ((uint32_t) inputString[32]) << (8 * 1);
      u_data.base |= ((uint32_t) inputString[31]) << (8 * 2);
      u_data.base |= ((uint32_t) inputString[30]) << (8 * 3);
      ff = u_data.real;

      yaw = cc;
      yaw-=yaw_calib2;
      incount++;
  
if(yaw_calib2==1000)
{
  yaw_calib2 = cc;
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

  }
  else if (serial_count > 38) //데이터가 10개 넘게 들어옴 (데inputString[19]이터 수신 실패)
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

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  UBRR1H = 0x00;
  UBRR1L = 0x08;
  UCSR1A = 0x00;
  UCSR1B = 0x98;
  UCSR1C = 0x06;

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
  delay(100);
 pinMode(RELAY0, OUTPUT);
  pinMode(LED_SIG, OUTPUT);

  pinMode(RTS, OUTPUT);
  GRIPPER_SERVO.attach(GRIPPER_SIG);
digitalWrite(RTS, HIGH);
  pwm_m = 1400;
  GRIPPER_SERVO.writeMicroseconds(pwm_m);
  
  
  delay(1000);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  /*
Serial.print(aa);
Serial.print(' ');
Serial.print(bb);
Serial.print(' ');
Serial.print(cc);
Serial.print(' ');
Serial.print(yaw);
Serial.print(' ');
Serial.print(dd);
Serial.print(' ');


    Serial.print(ee);
Serial.print(' ');
Serial.print(ff);
Serial.print(' ');
    Serial.print(inputString[19]);
Serial.print(' ');
    Serial.print(inputString[35]);
Serial.println(' ');

*/
if (Serial.available())
  {
    
    

     


    // get the new byte:
    char inChar = (char)Serial.read();
    if (inChar == 'q')
    {
      yaw_calib2 = 1000;
    }
    else if (inChar == 'w')
    {
    }
    else if (inChar == 'a')
    {
      digitalWrite(RELAY0, HIGH);
      delay(5000);
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
    else if (inChar == 'e')
    {
      pwm_m = 1800;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 'r')
    {
      pwm_m = 1400;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 't')
    {
      pwm_m = 1200;
      Serial.println(pwm_m);
      GRIPPER_SERVO.writeMicroseconds(pwm_m);
    }
    else if (inChar == 'g')
    {
      pwm_m0 = 1500;
      pwm_m1 = 1500;
      pwm_m2 = 1500;
      pwm_m3 = 1500;
      pwm_m4 = 1500;
      pwm_m5 = 1500;
    }
    else if (inChar == 'y')
    {
      pwm_m0 += 10;
    }
    else if (inChar == 'h')
    {
      pwm_m0 -= 10;
    }
    else if (inChar == 'u')
    {
      pwm_m1 += 10;
    }
    else if (inChar == 'j')
    {
      pwm_m1 -= 10;
    }
    else if (inChar == 'i')
    {
      pwm_m2 += 10;
    }
    else if (inChar == 'k')
    {
      pwm_m2 -= 10;
    }
    else if (inChar == 'o')
    {
      pwm_m3 += 10;
    }
    else if (inChar == 'l')
    {
      pwm_m3 -= 10;
    }
    else if (inChar == 'p')
    {
      pwm_m4 += 10;
    }
    else if (inChar == ';')
    {
      pwm_m4 -= 10;
    }
    else if (inChar == '[')
    {
      pwm_m5 += 10;
    }
    else if (inChar == '\'')
    {
      pwm_m5 -= 10;
    }
    Serial.print(pwm_m0);
    Serial.print(' ');
    Serial.print(pwm_m1);
    Serial.print(' ');
    Serial.print(pwm_m2);
    Serial.print(' ');
    Serial.print(pwm_m3);
    Serial.print(' ');
    Serial.print(pwm_m4);
    Serial.print(' ');
    Serial.println(pwm_m5);
    
    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }





delay(1000);
Serial.println(incount);
incount = 0;
}

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2) {
  uint8_t buf0[10] = {0xff,       0xff,       ID,         pwm0 / 256,
                      pwm0 % 256, pwm1 / 256, pwm1 % 256, pwm2 / 256,
                      pwm2 % 256, 0xfe};
  Serial2.write(buf0, 10);
}
