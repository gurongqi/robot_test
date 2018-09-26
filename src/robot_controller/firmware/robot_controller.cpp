/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <Arduino.h>
#include "PWM.h"
#include "include.h"
#include "LobotServoController.h"

ros::NodeHandle nh;
// ServoController servo_controller;
LobotServoController myse;

// void messageCb( const std_msgs::UInt16MultiArray& toggle_msg){
//   for (uint8 i=0;i<6;i++)
//   {
//   	uint8 servo_id = toggle_msg.data[0+3*i];
//   	uint16 servo_angle = toggle_msg.data[1+3*i];
//   	uint16 run_time = toggle_msg.data[2+3*i];
//   	servo_controller.ServoSetPluseAndTime(servo_id,servo_angle,run_time);
//   }
//   digitalWrite(13, HIGH-digitalRead(13));   // blink the led
// }

// ros::Subscriber<std_msgs::UInt16MultiArray> sub("toggle_led", &messageCb );

void messageCb( const std_msgs::UInt16& toggle_msg){
  uint16 servo_angle = toggle_msg.data;
  // myse.moveServo(1,servo_angle,1000);
  // servo_controller.ServoSetPluseAndTime(1,servo_angle,1000);

  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::UInt16> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  digitalWrite(13,HIGH);
  nh.initNode();
  nh.subscribe(sub);
  // servo_controller.InitPWM();
  myse.moveServo(0,1500,1000); //0号舵机1000ms移动至1500位置
  digitalWrite(13, HIGH-digitalRead(13));
  delay(1000);
  myse.moveServo(0,2000,1000); //0号舵机1000ms移动至1500位置
  digitalWrite(13, HIGH-digitalRead(13));
  delay(1000);
  myse.moveServo(0,1500,1000); //0号舵机1000ms移动至1500位置
  digitalWrite(13, HIGH-digitalRead(13));
  delay(1000);
  myse.moveServo(0,2000,1000); //0号舵机1000ms移动至1500位置
  digitalWrite(13, HIGH-digitalRead(13));
  delay(1000);
}

void loop()
{
  nh.spinOnce();
  // servo_controller.ServoPwmDutyCompare();
  delay(1);
}