// ROS node for SEPP - gateway to actions of SEPP where decitions are made
// 2018-09-18 by Andreas Lachenschmidt <a.lachenschmidt@web.de>
//
// Changelog:
//      2018-09-18 - initial commit

/* ============================================
SEPP ROS Nodes are placed under the MIT license
Copyright (c) 2018 Andreas Lachenschmidt

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Joy.h"

ros::Publisher decided_thrust_pub;

ros::Publisher decided_steer_pub;

ros::Publisher decided_direction_pub;

ros::Subscriber joy_sub;

ros::Subscriber ermergancyBreak_sub;

#define TRUST_MAP_INPUT_MIN -100000
#define TRUST_MAP_INPUT_MAX 100000
#define TRUST_MAP_OUTPUT_MIN 0
#define TRUST_MAP_OUTPUT_MAX 2000

#define STEER_MAP_INPUT_MIN -100000
#define STEER_MAP_INPUT_MAX 100000
#define STEER_MAP_OUTPUT_MIN 0
#define STEER_MAP_OUTPUT_MAX 2000

enum directions
{
  UNKNOWN,
  NUTRUAL,
  FORWARD_DIRECTION,
  REVERSE_DIRECTION,
  BREAK_DIRECTION
};
directions direction = NUTRUAL;

static bool emergancyBreakActive = false;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

directions evaluateDirection(int32_t forwardButton, int32_t reverseButton, directions currentDirection, float breakValue)
{
  directions evaluatedDirection = currentDirection;
  if (0.8 >= breakValue)
  {
    evaluatedDirection = BREAK_DIRECTION;
  }
  else if (1 == forwardButton && 0 == reverseButton)
  {
    evaluatedDirection = FORWARD_DIRECTION;
  }
  else if (0 == forwardButton && 1 == reverseButton)
  {
    evaluatedDirection = REVERSE_DIRECTION;
  }
  return evaluatedDirection;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{

  // thrust
  std_msgs::Int16 int16_msg_decided_thrust;

  int16_msg_decided_thrust.data = map((-1 * msg->axes[1] * 100000), TRUST_MAP_INPUT_MIN, TRUST_MAP_INPUT_MAX, TRUST_MAP_OUTPUT_MIN, TRUST_MAP_OUTPUT_MAX);

  decided_thrust_pub.publish(int16_msg_decided_thrust);

  //steer
  std_msgs::Int16 int16_msg_decided_steer;

  int16_msg_decided_steer.data = map(msg->axes[0] * 100000, STEER_MAP_INPUT_MIN, STEER_MAP_INPUT_MAX, STEER_MAP_OUTPUT_MIN, STEER_MAP_OUTPUT_MAX);

  decided_steer_pub.publish(int16_msg_decided_steer);

  //direction
  std_msgs::Int16 int16_msg_decided_direction;
  direction = evaluateDirection(msg->buttons[4], msg->buttons[5], direction, msg->axes[2]);
  int16_msg_decided_direction.data = direction;

  decided_direction_pub.publish(int16_msg_decided_direction);
}

void emergancyStopCallback(const std_msgs::Bool::ConstPtr &msg)
{
  emergancyBreakActive = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decider");

  ros::NodeHandle n;

  joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

  ermergancyBreak_sub = n.subscribe<std_msgs::Bool>("emergancyStop", 1, emergancyStopCallback);

  decided_thrust_pub = n.advertise<std_msgs::Int16>("decided_thrust", 1);

  decided_steer_pub = n.advertise<std_msgs::Int16>("decided_steer", 1);

  decided_direction_pub = n.advertise<std_msgs::Int16>("decided_direction", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (true == emergancyBreakActive)
    {
      direction = BREAK_DIRECTION;
    }
    ros::spin();
  }
  return 0;
}
