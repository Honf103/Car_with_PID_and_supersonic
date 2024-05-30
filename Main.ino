// PID constants
#include <Servo.h>

float kp = 77, ki = 0, kd = 160;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;
char state;
// Sensor values
int sensor[5] = {0, 0, 0, 0, 0};

int gia_tri_ban_dau = 85; // Base speed
int PID_phai, PID_trai;
char var;

int stop_distance = 6;// Khoảng cách phát hiện vật cản
const int trigPin = 12; // kết nối chân trig với chân 11 arduino
const int echoPin = 13; // kết nối chân echo với chân 12 arduino
long duration; 
int distance; 


int servoPin = 9;
Servo servo;
int angle = 0;
int speed;
// Motor control pins
#define in1 4
#define in2 5
#define in3 6
#define in4 7
#define ena 3
#define enb 11

// Function prototypes
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void dung(int);
void chay_thang(int);
void lui(int);
void phai(int,int);
void trai(int,int);

void setup() {
  // Initialize motor control pins
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);
  digitalWrite(8, HIGH);
    delay(150);
    digitalWrite(8, LOW);
    delay(100);
    digitalWrite(8, HIGH);
    delay(150);
    digitalWrite(8, LOW);
        delay(10);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  servo.write(90);
  delay(2000);
  digitalWrite(8, HIGH);
  delay(300);
  digitalWrite(8, LOW);
  delay(10);

}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  delay(15);
  read_sensor_values();
  if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) {
    lui(40);  
  }
  else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) {
    lui(50);  
    dung(100);
    chay_thang(60);
  }
   else {
    calculate_pid();
    motor_control();
  }
}

void read_sensor_values() {
  // Read sensor values
  sensor[0] = digitalRead(A0);
  sensor[1] = digitalRead(A1);
  sensor[2] = digitalRead(A2);
  sensor[3] = digitalRead(A3);
  sensor[4] = digitalRead(A4);
  // Calculate error based on sensor values
  if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[4]==1)&&(sensor[4]==0))
  error=4;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
  error=3;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==1))
  error=2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
  error=1;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=0;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-1;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-3;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
  error=-4;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==1))
    if(error==-4) error=-5;
    else error=5;
}

void calculate_pid() {
  // Calculate PID values
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (kp * P) + (ki * I) + (kd * D);
  previous_error = error;
}

void motor_control() {

    // Continue normal PID control for small adjustments
    if(distance > stop_distance){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    //PID_phai = gia_tri_ban_dau - PID_value;
    //PID_trai = gia_tri_ban_dau + PID_value;
    PID_phai = constrain(gia_tri_ban_dau - PID_value, 0, 121);//121, 115
    PID_trai = constrain(gia_tri_ban_dau + PID_value, 0, 121);
    analogWrite(enb, PID_phai);
    analogWrite(ena, PID_trai);
  }
  else
  {
    speed = 150;
    analogWrite(enb, 0);
    analogWrite(ena, 0);
    digitalWrite(8, HIGH);
    delay(300);
    digitalWrite(8, LOW);
    delay(50);
    servo.write(0);
    delay(1600);
    Serial.println("Davo");
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    delay(50);

    if(distance > stop_distance)
    {
      lui(200);
      dung(100);
      phai(400,speed);
      chay_thang(385);
      dung(100);
      trai(400,speed);
      chay_thang(100);
      dung(100);

      servo.write(90);
      delay(1600);
      read_sensor_values();

      while(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
      {
        speed = 200;
        trai(8,speed);
        read_sensor_values();

      }
      return;
    } 
    else
    {
      servo.write(180);
      delay(1600);
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;
      delay(50);

    if(distance > stop_distance)
    {
      lui(175);
      dung(100);
      trai(400,speed);
      chay_thang(385);
      dung(100);
      phai(350,speed);
      chay_thang(100);
      dung(100);
      servo.write(90);
      delay(1600);
      read_sensor_values();
      while(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
      {
        speed = 200;
        phai(8,speed);
        read_sensor_values();
      }
      return;
     }
    }
  }
}
void trai(int dl, int speed)
{
  digitalWrite (in1, LOW); // cho xe robot xoay trái 1 đoạn
  digitalWrite(in2, LOW);
  digitalWrite (in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, speed);
  analogWrite(enb, speed);
  delay(dl);
}
void phai(int dl, int speed)
{
  digitalWrite (in1, LOW); // cho xe robot xoay trái 1 đoạn
  digitalWrite(in2, HIGH);
  digitalWrite (in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, speed);
  analogWrite(enb, speed);
  delay(dl);
}
void dung(int dl) {
  // Stop the motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(dl);
}

void chay_thang(int dl) {
  // Move forward with base speed
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, gia_tri_ban_dau);
  analogWrite(enb, gia_tri_ban_dau);
  delay(dl);
}
void lui(int dl) {
  // Move forward with base speed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena, 110);
  analogWrite(enb, 110);
  delay(dl);
}
