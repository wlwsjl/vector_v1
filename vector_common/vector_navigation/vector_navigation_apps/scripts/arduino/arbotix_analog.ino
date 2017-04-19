/*
 * This file polls a single MaxBotix MB 1230 sonar and publishes data  
 * as a Range message.
 * This is not a driver, but could be modified and called by a driver in the event 
 * that multiple devices are being polled by the arduino.
 *
 * NOTE: we assume the input voltage is 5V and not 3V, which requires a different conversion factor
 * 
 * Author: Maxwell Svetlik 2017
 */
 
#include <ros.h>
#include <sensor_msgs/Range.h>

sensor_msgs::Range sonar_msg;
ros::Publisher pub_sonar("/sonar", &sonar_msg);
ros::NodeHandle nh;

const int anPin = 0;
long anVolt, mm;

void setup() {
  nh.initNode();
  nh.advertise(pub_sonar);
  Serial.begin(9600);

  sonar_msg.radiation_type = sonar_msg.ULTRASOUND;
  sonar_msg.min_range = 0.2;
  sonar_msg.max_range = 7.0;
  sonar_msg.header.frame_id = "sonar_link";
  sonar_msg.field_of_view = 0.306;
}

void read_sensor(){
  anVolt = analogRead(anPin);
  mm = anVolt * 10;
}

//serial monitor debug
void print_range(){
  Serial.print("S1");
  Serial.print("=");
  Serial.print(mm);
}

void loop() {
  read_sensor();
  sonar_msg.range = float(mm)/1000.0;
  pub_sonar.publish(&sonar_msg);
  //print_range();
  delay(100);
  nh.spinOnce();
} 
