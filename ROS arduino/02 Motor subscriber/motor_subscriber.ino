#include<ros.h>
#include<geometry_msgs/Twist.h>

ros::NodeHandle nh;

int STBY = A4;
int AIN1 = 7;
int AIN2 = 8;
int BIN1 = 9;
int BIN2 = 10;

int PWMA = 11;
int PWMB = 3;

// wheel speed
int SPD_A = 220;
int SPD_B = 180;



void forward()
{
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, SPD_A);
  analogWrite(PWMB, SPD_B);
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void reverse()
{
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, SPD_A);
  analogWrite(PWMB, SPD_B);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void brake()
{
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, SPD_A);
  analogWrite(PWMB, SPD_B);
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void left()
{
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, SPD_A);
  analogWrite(PWMB, SPD_B);
  
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void right()
{
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, SPD_A);
  analogWrite(PWMB, SPD_B);
  
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// ROS callback function
void messageCallback(const geometry_msgs::Twist& msg){


     if (msg.linear.x > 0.0 && msg.angular.z == 0) // forward
     {
      forward();
     }
     else if (msg.linear.x < 0.0 && msg.angular.z == 0) // reverse
     {
      reverse();
     }
     else if (msg.linear.x == 0.0 && msg.angular.z == 0) // stop
     {
      brake();
     }
     else if (msg.linear.x == 0.0 && msg.angular.z < 0) // turn right
     {
      right();
     }
     else if (msg.linear.x == 0.0 && msg.angular.z > 0) // turn left
     {
      left();
     }
     else // commands to turn and move forward at the same time will not work
     {
       brake();
     }

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", &messageCallback ); // create subscriber

void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  Serial.begin(9600);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
