// PID constants
#include <Servo.h>

float kp = 50.2, ki = 0.01, kd = 160;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0;
char state;
// Sensor values
int sensor[5] = {0, 0, 0, 0, 0};

int gia_tri_ban_dau = 90; // Base speed
int PID_phai, PID_trai;
char var;

int stop_distance = 20;// Khoảng cách phát hiện vật cản
const int trigPin = 12; // kết nối chân trig với chân 11 arduino
const int echoPin = 13; // kết nối chân echo với chân 12 arduino
long duration; 
int value; 


int servoPin = 9;
Servo servo;
int angle = 0;
int speed;
int k,sum,old_sum,distance;
// Motor control pins
#define in1 4
#define in2 5
#define in3 6
#define in4 7
#define ena 3
#define enb 11

// Function prototypes
uint16_t read_sensor_values();
void calculate_pid(int);
void motor_control();
void dung(int);
void chay_thang(int);
void lui(int);
void phai(int,int);
void trai(int,int);
void vatcan();
void tranhvatcan();
void xoaytronphai(int);
void xoaytrontrai(int);

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
  
  vatcan();
  if(distance < stop_distance)
  {
    tranhvatcan();
  }
  else
  {
  value = read_sensor_values();
  /////////////////
  calculate_pid(value);
  //////////////////
  motor_control();
  }
  /*
  Serial.println(PID_phai);
  Serial.println(PID_trai);
  */
  delay(10);
}
void vatcan()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}
uint16_t read_sensor_values() {
  // Read sensor values
  sum = 0;
  k = 0;

  sensor[2] = digitalRead(A0);
  sensor[1] = digitalRead(A1);
  sensor[0] = digitalRead(A2);
  //Serial.print(sensor[0]);Serial.print(" ");Serial.print(sensor[1]);Serial.print(" ");Serial.print(sensor[2]);Serial.println(" ");
  // Calculate error based on sensor values
  for(int i = 1; i < 4; i++)
  {
    if(sensor[i-1] == 1)
    {
      k = k+1;
      sum = sum + i*100;
    }
  }
  if( k>0 && k<4)
  {
    old_sum = sum/k;
    return old_sum;
  }
  else
  {
    if(old_sum == 200)
    {
      return 200;
    }
    else 
    {
      return old_sum;
    }
  }
}

void calculate_pid(int value) {
  // Calculate PID values
  error = (200 - value)/23;
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (kp * P) + (ki * I) + (kd * D);
  previous_error = error;
}

void motor_control() {

    // Continue normal PID control for small adjustments
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    //PID_phai = gia_tri_ban_dau - PID_value;
    //PID_trai = gia_tri_ban_dau + PID_value;
    PID_phai = constrain(gia_tri_ban_dau - PID_value,0,200);//121, 115
    PID_trai = constrain(gia_tri_ban_dau + PID_value,0,200);
    analogWrite(enb, PID_phai);
    analogWrite(ena, PID_trai);
}
void tranhvatcan()
{
    speed = 150;
    analogWrite(enb, 0);
    analogWrite(ena, 0);
  for(int i =0; i <5; i++)
    {
      digitalWrite(8, HIGH);
      delay(100);
      digitalWrite(8, LOW);
      delay(50);
    }
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
      lui(300);
      xoaytronphai(500);
      sensor[2] = digitalRead(A0);
      sensor[1] = digitalRead(A1);
      sensor[0] = digitalRead(A2);
      while(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0)
      {
        xoaytronphai(10);
        sensor[2] = digitalRead(A0);
        sensor[1] = digitalRead(A1);
        sensor[0] = digitalRead(A2);
      }
      analogWrite(enb, 0);
      analogWrite(ena, 0);
      servo.write(90);
      delay(1600);
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
      lui(300);
      xoaytrontrai(500);
      sensor[2] = digitalRead(A0);
      sensor[1] = digitalRead(A1);
      sensor[0] = digitalRead(A2);      
      while(sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0)
      {
        xoaytrontrai(10);
      sensor[2] = digitalRead(A0);
      sensor[1] = digitalRead(A1);
      sensor[0] = digitalRead(A2);     
      }
      analogWrite(enb, 0);
     analogWrite(ena, 0);
      servo.write(90);
      delay(1600);
      return;
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
void xoaytrontrai(int dl)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ena, 130);
  analogWrite(enb, 130);
  delay(dl);
}
void xoaytronphai(int dl)
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ena, 130);
  analogWrite(enb, 130);
  delay(dl);
}
