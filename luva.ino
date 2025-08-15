#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>



//#include <SoftwareSerial.h>
#include "LobotServoController.h"
#include "MPU6050.h"
//#include "Wire.h"

// ROS Publishers and Messages
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
const long publish_interval = 100; // Corresponds to 10 Hz


//#define BTH_RX 11
//#define BTH_TX 12
float min_list[5] = {0, 0, 0, 0, 0};
float max_list[5] = {255, 255, 255, 255, 255};
float sampling[5] = {0, 0, 0, 0, 0};
float data[5] = {1500, 1500, 1500, 1500, 1500};
uint16_t ServePwm[5] = {1500, 1500, 1500, 1500, 1500};
uint16_t ServoPwmSet[5] = {1500, 1500, 1500, 1500, 1500};
bool turn_on = true;
//SoftwareSerial Bth(BTH_RX, BTH_TX);
//LobotServoController lsc(Bth);
float float_map(float in, float left_in, float right_in, float left_out, float right_out) {
  return (in - left_in) * (right_out - left_out) / (right_in - left_in) + left_out;
}
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax0, ay0, az0;
float gx0, gy0, gz0;
float ax1, ay1, az1;
float gx1, gy1, gz1;
float radianX;
float radianY;
float radianZ;
float radianX_last;
float radianY_last;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int mode = 0;
bool button_state = false;

// Function to handle the button press in a non-blocking way
void handleButton() {
  if (turn_on == false) {
    if (digitalRead(7) == false && button_state == false) {
      button_state = true;
      if (mode == 5) {
        mode = 0;
      } else {
        mode++;
      }
    }
    if (digitalRead(7) == true && button_state == true) {
      button_state = false;
    }
  }
}

// Function to control the LEDs based on the mode
void updateLEDs() {
  if (turn_on == false) {
    if (mode == 0) {
      digitalWrite(2, HIGH);
      digitalWrite(3, HIGH);
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
    } else if (mode == 1) {
      digitalWrite(2, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
    } else if (mode == 2) {
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
    } else if (mode == 3) {
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(5, HIGH);
      digitalWrite(6, HIGH);
    } else if (mode == 4) {
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(6, HIGH);
    }
  }
}

// Function to get data from each finger's potentiometer
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

 // NEW: Function to check if a single finger is lifted
void checkSingleFinger() {
  const int FINGER_THRESHOLD = 500;

  // Publish the boolean result for each finger individually
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


// Function to control LEDs based on finger movement thresholds

void controlFingerLEDs() {

const int LED_PINS[] = {2, 3, 4, 5, 6};

const int FINGER_THRESHOLD = 500; // The threshold for the finger reading

// Iterate through each of the 5 fingers

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
  pinMode(A6, INPUT); // It's good practice to declare all pins
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Wire.begin();
  Wire.setClock(20000); // Note: 200kHz is usually 200000L. Check your sensor's specs.
  accelgyro.initialize();

  //ROS
  nh.initNode();
  
  // Advertise all your publishers
  nh.advertise(thumb_finger_pub);
  nh.advertise(index_finger_pub);
  nh.advertise(middle_finger_pub);
  nh.advertise(ring_finger_pub);
  nh.advertise(pinky_finger_pub);
  nh.advertise(twist_pub);

}

void loop() {

  
  finger();
  handleButton();
  updateLEDs();

  if (millis() - last_publish_time > publish_interval) {
    last_publish_time = millis(); // Update the time

    checkSingleFinger(); // This function contains all your publish calls

    // You could also move the twist publish call here if needed
    twist_pub.publish(&twist_msg); 
  }  
  checkSingleFinger();
  controlFingerLEDs();
  //terminal code: rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
  nh.spinOnce();
  
}