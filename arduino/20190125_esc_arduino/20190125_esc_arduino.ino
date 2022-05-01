#include <Servo.h>

#define ID 0x02

#define PWM0 9
#define PWM1 10
#define PWM2 11

#define RTS 2

Servo PWM0_SERVO;
Servo PWM0_SERV1;
Servo PWM0_SERV2;

int pwm_m0 = 1500;
int pwm_m1 = 1500;
int pwm_m2 = 1500;

void setup() {
  Serial.begin(115200);

  pinMode(RTS, OUTPUT);
  
  PWM0_SERVO.attach(PWM0);
  PWM0_SERV1.attach(PWM1);
  PWM0_SERV2.attach(PWM2);

  digitalWrite(RTS, LOW);
  // put your setup code here, to run once:

  delay(1000);
}

void loop() {
uint8_t buf0[10];
Serial.readBytes(buf0,10);

 if (buf0[9]==0xfe) {
      if (buf0[0] == 0xff &&
          buf0[1] == 0xff && buf0[2] == ID &&

          buf0[9] == 0xfe) //데이터 수신 성공
      {
        pwm_m0 = 256*buf0[3]+buf0[4];
        pwm_m1 = 256*buf0[5]+buf0[6];
        pwm_m2 = 256*buf0[7]+buf0[8];

 PWM0_SERVO.writeMicroseconds(pwm_m0);
 PWM0_SERV1.writeMicroseconds(pwm_m1);
 PWM0_SERV2.writeMicroseconds(pwm_m2);
      }
    // clear the string:

    buf0[9] = 0x00;
  }
}
