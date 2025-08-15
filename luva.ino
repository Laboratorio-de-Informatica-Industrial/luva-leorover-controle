#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


#include "LobotServoController.h"

ros::NodeHandle nh;

geometry_msgs::Twist twist_msg;
ros::Publisher twist_pub("leo_main_duque/cmd_vel", &twist_msg);
std_msgs::Bool thumb_finger_msg;
std_msgs::Bool index_finger_msg;
std_msgs::Bool middle_finger_msg;
std_msgs::Bool ring_finger_msg;
std_msgs::Bool pinky_finger_msg;
ros::Publisher thumb_finger_pub("thumb_finger_msg", &thumb_finger_msg);
ros::Publisher index_finger_pub("index_finger_msg", &index_finger_msg);
ros::Publisher middle_finger_pub("middle_finger_msg", &middle_finger_msg);
ros::Publisher ring_finger_pub("ring_finger_msg", &ring_finger_msg);
ros::Publisher pinky_finger_pub("pinky_finger_msg", &pinky_finger_msg);

unsigned long last_publish_time = 0;
const long publish_interval = 100;


float min_list[5] = {0, 0, 0, 0, 0};
float max_list[5] = {255, 255, 255, 255, 255};
float sampling[5] = {0, 0, 0, 0, 0};
float data[5] = {1500, 1500, 1500, 1500, 1500};
bool turn_on = true;
float float_map(float in, float left_in, float right_in, float left_out, float right_out) {
  return (in - left_in) * (right_out - left_out) / (right_in - left_in) + left_out;
}

void finger() {
  for (int i = 14; i <= 18; i++) {
    if (i < 18)
      sampling[i - 14] = analogRead(i);
    else
      sampling[i - 14] = analogRead(A6);
    data[i - 14 ] = float_map(sampling[i - 14], min_list[i - 14], max_list[i - 14], 2500, 500);
    data[i - 14] = data[i - 14] > 2500 ? 2500 : data[i - 14];
    data[i - 14] = data[i - 14] < 500 ? 500 : data[i - 14];
  }
}
void checkSingleFinger() {
  const int FINGER_THRESHOLD = 500;


  thumb_finger_msg.data = (sampling[0] < FINGER_THRESHOLD);
  thumb_finger_pub.publish(&thumb_finger_msg);
  
  index_finger_msg.data = (sampling[1] < FINGER_THRESHOLD);
  index_finger_pub.publish(&index_finger_msg);

  middle_finger_msg.data = (sampling[2] < FINGER_THRESHOLD);
  middle_finger_pub.publish(&middle_finger_msg);

  ring_finger_msg.data = (sampling[3] < FINGER_THRESHOLD);
  ring_finger_pub.publish(&ring_finger_msg);

  pinky_finger_msg.data = (sampling[4] < FINGER_THRESHOLD);
  pinky_finger_pub.publish(&pinky_finger_msg);



  if (sampling[0] < FINGER_THRESHOLD && sampling[1] > FINGER_THRESHOLD && sampling[2] > FINGER_THRESHOLD && sampling[3] > FINGER_THRESHOLD && sampling[4] > FINGER_THRESHOLD){
    twist_msg.angular.z = 1; //ir para a esquerda
    twist_msg.linear.x = 0;
    twist_pub.publish(&twist_msg);
    }else if (sampling[0] > FINGER_THRESHOLD && sampling[1] > FINGER_THRESHOLD && sampling[2] > FINGER_THRESHOLD && sampling[3] > FINGER_THRESHOLD && sampling[4] < FINGER_THRESHOLD){
    twist_msg.angular.z = -1; //ir para a direita
    twist_msg.linear.x = 0;
    twist_pub.publish(&twist_msg);
    }else if (sampling[0] > FINGER_THRESHOLD && sampling[1] < FINGER_THRESHOLD && sampling[2] > FINGER_THRESHOLD && sampling[3] > FINGER_THRESHOLD && sampling[4] > FINGER_THRESHOLD){
    twist_msg.angular.z = 0; //ir para a frente
    twist_msg.linear.x = 1;
    twist_pub.publish(&twist_msg);
    } else if (sampling[0] > FINGER_THRESHOLD && sampling[1] > FINGER_THRESHOLD && sampling[2] > FINGER_THRESHOLD && sampling[3] > FINGER_THRESHOLD && sampling[4] > FINGER_THRESHOLD){
    twist_msg.angular.z = 0; //ir para trás
    twist_msg.linear.x = -1;
    twist_pub.publish(&twist_msg);
    }else if (sampling[0] < FINGER_THRESHOLD && sampling[1] < FINGER_THRESHOLD && sampling[2] < FINGER_THRESHOLD && sampling[3] < FINGER_THRESHOLD && sampling[4] < FINGER_THRESHOLD){
    twist_msg.angular.z = 0; //ficar parado
    twist_msg.linear.x = 0;
    twist_pub.publish(&twist_msg);
    }else{
      twist_msg.angular.z = 0;
      twist_msg.linear.x = 0;
      twist_pub.publish(&twist_msg);
    }
}


//Função que controla o LED

void controlFingerLEDs() {

const int LED_PINS[] = {2, 3, 4, 5, 6};

const int FINGER_THRESHOLD = 500; 



for (int i = 0; i < 5; i++){

  if (sampling[i] > FINGER_THRESHOLD) {
    digitalWrite(LED_PINS[i], HIGH);
  } else {
    digitalWrite(LED_PINS[i], LOW);
  }
}

}

void setup() {

  pinMode(7, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A6, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  //ROS
  nh.initNode();
  nh.advertise(thumb_finger_pub);
  nh.advertise(index_finger_pub);
  nh.advertise(middle_finger_pub);
  nh.advertise(ring_finger_pub);
  nh.advertise(pinky_finger_pub);
  nh.advertise(twist_pub);

}

void loop() {

  finger();

  if (millis() - last_publish_time > publish_interval) {
    last_publish_time = millis();

    checkSingleFinger(); 

    twist_pub.publish(&twist_msg); 
  }  
  checkSingleFinger();
  controlFingerLEDs();
  //terminal code: rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
  nh.spinOnce();
  
}