#include<ros.h>
#include<geometry_msgs/Twist.h>

ros::NodeHandle nh;

void messageCallback(const geometry_msgs::Twist& msg){
  if(msg.linear.x > 0.1){
    digitalWrite(13, HIGH);
  }
  else
    digitalWrite(13, LOW);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel_mux/input/teleop", &messageCallback );

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
