#include <ros.h>
#include <melle/MellE_msg.h>
#include <melle/PC_msg.h>
#include <melle/AndroidSensorData.h>
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
RoboClaw motor_control(&Serial2, 10000);
PID test = PID(0, 10, 0.01, 0, 0, 15, -15);
PID_horz test_horz = PID_horz(0, 10, 0.01, 0, 0, 20, -20);
//End motor stuff

//GPS Stuff
TinyGPSPlus gps;
float curr_lat;
float curr_long;
float dis_to_dest = 0;
float curr_course;
float dest_course = 0;
float des_lat;
float des_long;
bool obstacle;
bool keyboard;
int sats;
//End GPS stuff

//ROS Stuff
ros::NodeHandle arduinoNode;

melle::MellE_msg msg_to_send;
melle::PC_msg msg_received;
melle::Init_stat_msg init_stat;

ros::Publisher MellE_pub("MellE_msg", &msg_to_send);
ros::Publisher Init_stat_pub("Init_stat_msg", &init_stat);

//Keyboard teleop
void motor_com_cb(const geometry_msgs::Twist& in_msg)
{
  if (!keyboard)
  {
    return;
  }
  if (in_msg.linear.x > 0)
  {
    motor_control.ForwardMixed(mc_address, (int) in_msg.linear.x);
  }
  if (in_msg.linear.x < 0)
  {
    motor_control.BackwardMixed(mc_address, (int) - 1 * in_msg.linear.x);
  }
  if (in_msg.angular.z < 0)
  {
    motor_control.TurnLeftMixed(mc_address, -1 * (int)in_msg.angular.z * (int)in_msg.linear.z);
  }
  if (in_msg.angular.z > 0)
  {
    motor_control.TurnRightMixed(mc_address, (int)in_msg.angular.z * (int)in_msg.linear.z);
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
  if(keyboard==1)
  {
    current_state=KEYBOARD;
  }
  else if (obstacle == true)
  {
    current_state = OBSTACLE_AVOIDANCE;
  }
  else if(current_state!=WAIT_FOR_GPS_LOCK)
  {
    current_state=MOVE_TO_WAYPOINT;
    msg_to_send.waypoint_reached=0;
  }
}
ros::Subscriber <melle::PC_msg> pc_msg_subs("PC_msg", &waypoint_callback);
// end of dest lat long

//Sensor data
void sensor_data_cb (const melle::AndroidSensorData& msg)
{
  curr_lat = msg.latitude;
  curr_long = msg.longitude;
  curr_course = msg.azimuth;
  sats = msg.sats;
  if (sats == 0)
  {
    init_stat.gps_status = 0;
    current_state = WAIT_FOR_GPS_LOCK;
  }
  else if(current_state!=MOVE_TO_WAYPOINT)
  {
    init_stat.gps_status = 1;
    current_state = GET_WAYPOINT;
  }
  if(current_state==MOVE_TO_WAYPOINT)
  {
    dis_to_dest=gps.distanceBetween(curr_lat,curr_long,des_lat,des_long);
    dest_course=gps.courseTo(curr_lat,curr_long,des_lat,des_long);
  }
}
ros::Subscriber <melle::AndroidSensorData> position_sub("AndroidSensorData", &sensor_data_cb);
//end of sensor data

void obstacle_avoid()
{
  //Obstacle avoidance routine goes here
  motor_control.TurnRightMixed(mc_address, 0);
  motor_control.ForwardMixed(mc_address, 0);
}

void move_to_waypoint()
{
  msg_to_send.dist_to_dest=dis_to_dest;
  msg_to_send.heading_to_dest=dest_course;
  float newSpeed = test.getNewValue(curr_course, dest_course, elapsedTime);
  float newSpeedHorz = test_horz.getNewValue(dis_to_dest, elapsedTime);
  if (abs(curr_course - dest_course) > 10)
  {
    motor_control.ForwardMixed(mc_address, 0);
    if (newSpeed >= 0)
    {
      motor_control.TurnLeftMixed(mc_address, newSpeed);
    }
    else
    {
      motor_control.TurnRightMixed(mc_address, -1 * newSpeed);
    }
  }
  else
  {
    motor_control.TurnLeftMixed(mc_address, 0);
    if (newSpeedHorz >= 0)
    {
      motor_control.ForwardMixed(mc_address, newSpeedHorz);
    }
    else
    {
      motor_control.BackwardMixed(mc_address, -1 * newSpeedHorz);
    }
  }
  if (dis_to_dest < 2 && abs(curr_course - dest_course) < 10)
  {
    msg_to_send.waypoint_reached=1;
    current_state=GET_WAYPOINT;
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
  init_stat.gps_status=false;
  current_state=WAIT_FOR_GPS_LOCK;
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
  elapsedTime = (float)(millis() - oldTime);
  oldTime = millis();
  msg_to_send.current_state=current_state;
  MellE_pub.publish(&msg_to_send);
  Init_stat_pub.publish(&init_stat);
  arduinoNode.spinOnce();
  delay(100);
}
