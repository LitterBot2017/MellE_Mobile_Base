#include <ros.h>
#include <melle/MellE_msg.h>
#include <melle/PC_msg.h>
#include <melle/AndroidSensorData.h>
#include <melle_obstacle_avoidance/ObAvData.h>
#include <melle/Init_stat_msg.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <RoboClaw.h>
#include "TinyGPS++.h"
#include "PID.h"
#include "PID_horz.h"
#include "VoltageSensor.h"
#include "IRSensor.h"

//States
#define WAIT_FOR_GPS_LOCK 1
#define GET_WAYPOINT 2
#define MOVE_TO_WAYPOINT 3
#define OBSTACLE_AVOIDANCE 4
#define KEYBOARD 5
int current_state = 1;
//End of states

//Time
float elapsedTime = millis();
long oldTime = millis();
//End of Time

//Motor stuff
#define mc_address 0x81
#define MAX_TURNING_SPEED 20
#define MAX_FORWARD_SPEED 50
RoboClaw motor_control(&Serial2, 10000);
PID test = PID(0, 10, 0.01, 0, 0, MAX_TURNING_SPEED, -MAX_TURNING_SPEED);
PID_horz test_horz = PID_horz(0, 10, 0.01, 0, 0, MAX_FORWARD_SPEED, -MAX_FORWARD_SPEED);
//End motor stuff
long test_time = millis();

//GPS Stuff
TinyGPSPlus gps;
float curr_lat;
float curr_long;
float dis_to_dest = 1000;
float curr_course;
float dest_course = 100;
float des_lat;
float des_long;
bool obstacle;
bool keyboard;
int sats;
int waypointId;
//End GPS stuff

//obstacle avoidance
int obstacle_command=0;
//end obstacle avoidance

//ROS Stuff
ros::NodeHandle arduinoNode;

melle::MellE_msg msg_to_send;
melle::PC_msg msg_received;
melle::Init_stat_msg init_stat;

ros::Publisher MellE_pub("MellE_msg", &msg_to_send);
ros::Publisher Init_stat_pub("Init_stat_msg", &init_stat);

int voltageSensorPin = A0;
VoltageSensor voltageSensor = VoltageSensor(voltageSensorPin);

int irSensorPin = A3;
IRSensor irSensor(irSensorPin);

//Keyboard teleop
void motor_com_cb(const geometry_msgs::Twist& in_msg)
{
  if (current_state != KEYBOARD)
  {
    return;
  }
  if (in_msg.linear.x > 0)
  {
    motor_control.ForwardMixed(mc_address, 30);//(int) in_msg.linear.x);
  }
  if (in_msg.linear.x < 0)
  {
    motor_control.BackwardMixed(mc_address, 30);//(int) - 1 * in_msg.linear.x);
  }
  if (in_msg.angular.z < 0)
  {
    motor_control.TurnLeftMixed(mc_address, 30);//-1 * (int)in_msg.angular.z * (int)in_msg.linear.x);
  }
  if (in_msg.angular.z > 0)
  {
    motor_control.TurnRightMixed(mc_address, 30);//(int)in_msg.angular.z * (int)in_msg.linear.x);
  }
  if (in_msg.angular.z == 0 && in_msg.linear.x == 0)
  {
    motor_control.ForwardMixed(mc_address, 0);
    motor_control.TurnRightMixed(mc_address, 0);
  }
}

ros::Subscriber <geometry_msgs::Twist> keyboard_sub("cmd_vel", &motor_com_cb);
//End of keyboard teleop

//PC Messages with dest lat and long
void waypoint_callback (const melle::PC_msg& msg)
{
  des_lat = msg.lattitude;
  des_long = msg.longitude;
  obstacle = msg.obstacle;
  keyboard = msg.keyboard_activate;
  waypointId = msg.waypoint_id;
  if (keyboard == 1)
  {
    current_state = KEYBOARD;
  }
  else if (obstacle == true)
  {
    current_state = OBSTACLE_AVOIDANCE;
  }
  else if (current_state == GET_WAYPOINT)
  {
    current_state = MOVE_TO_WAYPOINT;
    msg_to_send.waypoint_reached = 0;
  }
  else if(!keyboard && !obstacle) 
  {
    if(sats>0 && current_state == MOVE_TO_WAYPOINT)
    {
      current_state=MOVE_TO_WAYPOINT;
    }
    else if(sats>0 && current_state != MOVE_TO_WAYPOINT)
    {
      current_state=GET_WAYPOINT;
    }
    else
    {
      current_state=WAIT_FOR_GPS_LOCK;
    }
  }
}
ros::Subscriber <melle::PC_msg> pc_msg_subs("PC_msg", &waypoint_callback);
// end of dest lat long

//Sensor data
void sensor_data_cb (const melle::AndroidSensorData& msg)
{
  if(keyboard||obstacle)
  {
    return;
  }
  curr_lat = msg.latitude;
  curr_long = msg.longitude;
  curr_course = msg.azimuth;
  sats = msg.sats;
  if (sats == 0)
  {
    init_stat.gps_status = 0;
    current_state = WAIT_FOR_GPS_LOCK;
  }
  else if (sats > 0 && current_state == WAIT_FOR_GPS_LOCK)
  {
    init_stat.gps_status = 1;
    current_state = GET_WAYPOINT;
  }
  if (current_state == MOVE_TO_WAYPOINT)
  {
    dis_to_dest = gps.distanceBetween(curr_lat, curr_long, des_lat, des_long);
    dest_course = gps.courseTo(curr_lat, curr_long, des_lat, des_long);
    if (dis_to_dest > 4)
    {
      msg_to_send.waypoint_reached = 0;
    }
  }
}
ros::Subscriber <melle::AndroidSensorData> position_sub("AndroidSensorData", &sensor_data_cb);
//end of sensor data

//obstacle commands
void ob_av_data_callback (const melle_obstacle_avoidance::ObAvData& msg) {
  obstacle_command=msg.command;
}

ros::Subscriber <melle_obstacle_avoidance::ObAvData> ob_av_data_sub("ob_av_data", &ob_av_data_callback);
//end of obstacles command

void obstacle_avoid()
{
  //Obstacle avoidance routine goes here
  switch (obstacle_command) {
    case 0:
      motor_control.TurnRightMixed(mc_address, 0);
      motor_control.BackwardMixed(mc_address, 20);
      break;
    case 1:
      motor_control.TurnLeftMixed(mc_address, 20);
      break;
    case 2:
      motor_control.TurnRightMixed(mc_address, 20);
      break;
    case 3:
      motor_control.TurnRightMixed(mc_address, 0);
      motor_control.BackwardMixed(mc_address, 0);
      break;
  }
}

void move_to_waypoint()
{
  msg_to_send.dist_to_dest = dis_to_dest;
  msg_to_send.heading_to_dest = dest_course;
//  if (curr_course > 270 && dest_course < 90) {
//    curr_course -= 360;
//  } else if (curr_course < 90 && dest_course > 270) {
//    curr_course += 360;
//  }
  float newSpeed = test.getNewValue(curr_course, dest_course, elapsedTime);
  float newSpeedHorz = test_horz.getNewValue(dis_to_dest, elapsedTime);
  newSpeed=(newSpeed/128)*64;
  newSpeedHorz=(newSpeedHorz/128)*64;
  float m1Speed=64+newSpeed+newSpeedHorz;
  float m2Speed=64-newSpeed+newSpeedHorz;
  if(m1Speed>89)
  {
    m1Speed=89;
  }
  if(m1Speed<39)
  {
    m1Speed=39;
  }
  if(m2Speed>89)
  {
    m2Speed=89;
  }
  if(m2Speed<39)
  {
    m2Speed=39;
  }
  motor_control.ForwardBackwardM1(mc_address,m1Speed);
  motor_control.ForwardBackwardM2(mc_address,m2Speed);
//  if (abs(curr_course - dest_course) > 30)
//  {
//    motor_control.ForwardMixed(mc_address, 0);
//    if (newSpeed >= 0)
//    {
//      motor_control.TurnLeftMixed(mc_address, newSpeed);
//    }
//    else
//    {
//      motor_control.TurnRightMixed(mc_address, -1 * newSpeed);
//    }
//  }
//  else
//  {
//    motor_control.TurnLeftMixed(mc_address, 0);
//    if (newSpeedHorz >= 0)
//    {
//      motor_control.ForwardMixed(mc_address, newSpeedHorz);
//    }
//    else
//    {
//      motor_control.BackwardMixed(mc_address, -1 * newSpeedHorz);
//    }
//  }
  if (dis_to_dest < 4 && current_state == MOVE_TO_WAYPOINT)
  {
    msg_to_send.waypoint_reached = 1;
    dis_to_dest = 1000;
    current_state = GET_WAYPOINT;
    motor_control.ForwardMixed(mc_address, 0);
    motor_control.TurnLeftMixed(mc_address, 0);
  }
}

void wait_for_gps_lock()
{
  motor_control.TurnRightMixed(mc_address, 0);
  motor_control.ForwardMixed(mc_address, 0);
}

void get_waypoint()
{
  motor_control.ForwardMixed(mc_address, 0);
  motor_control.TurnLeftMixed(mc_address, 0);
}

void setup() {
  // put your setup code here, to run once:
  // ROS Initialization
  arduinoNode.initNode();
  arduinoNode.advertise(MellE_pub);
  arduinoNode.advertise(Init_stat_pub);
  arduinoNode.subscribe(pc_msg_subs);
  arduinoNode.subscribe(position_sub);
  arduinoNode.subscribe(keyboard_sub);
  arduinoNode.subscribe(ob_av_data_sub);
  init_stat.gps_status = false;
  current_state = WAIT_FOR_GPS_LOCK;
  motor_control.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (current_state)
  {
    case OBSTACLE_AVOIDANCE:
      obstacle_avoid();
      break;
    case MOVE_TO_WAYPOINT:
      move_to_waypoint();
      break;
    case WAIT_FOR_GPS_LOCK:
      wait_for_gps_lock();
      break;
    case GET_WAYPOINT:
      get_waypoint();
      break;
    case KEYBOARD:
      break;
  }
  msg_to_send.battery = voltageSensor.getBatteryPercentage();
  msg_to_send.bin_fullness = irSensor.getBinFullnessPercentage();;
  msg_to_send.dist_to_dest = dis_to_dest;
  msg_to_send.heading_to_dest = dest_course;
  msg_to_send.current_waypoint_id = waypointId;
  elapsedTime = (float)(millis() - oldTime);
  oldTime = millis();
  msg_to_send.current_state = current_state;
  MellE_pub.publish(&msg_to_send);
  Init_stat_pub.publish(&init_stat);
  arduinoNode.spinOnce();
  delay(100);
}
