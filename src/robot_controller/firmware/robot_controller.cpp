/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Arduino.h>
#include "PWM.h"
#include "include.h"

ros::NodeHandle nh;
ServoController servo_controller;

void messageCb( const std_msgs::UInt16MultiArray& toggle_msg){
  uint8 servo_id=toggle_msg.data[0];
  uint16 servo_angle=toggle_msg.data[1];
  uint16 run_time = toggle_msg.data[2];
  servo_controller.ServoSetPluseAndTime(servo_id,servo_angle,run_time);
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  servo_controller.InitPWM();
}

void loop()
{
  nh.spinOnce();
  servo_controller.ServoPwmDutyCompare();
  delay(1);
}