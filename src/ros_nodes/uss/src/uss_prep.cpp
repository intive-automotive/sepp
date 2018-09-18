// ROS node for SEPP - prepare inputs from ultrasonic sensors of SEPP
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

#include <string.h>
#include <sstream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

using namespace std;

class UltrasonicProcessing
{
  ros::Subscriber uss_sub_;
  ros::Publisher uss_pub_;
  stringstream ss_;
  sensor_msgs::Range range_msg_preprocessed_uss_;
  static const float rangeLimit = 0.4;
  ros::NodeHandle nh_;
  ros::Publisher emergancyStop_pub_;
  std_msgs::Bool emergancyStop_msg_preprocessed_uss_;

public:
  UltrasonicProcessing(string topic, ros::NodeHandle nodeHanlde)
  {
    nh_ = nodeHanlde;
    ss_ << topic << "_preprocessed";
    //emergancy Stop
    emergancyStop_pub_ = nh_.advertise<std_msgs::Bool>("emergancyStop", 1);
    // Subscrive to input video feed and publish output video feed
    uss_sub_ = nh_.subscribe<sensor_msgs::Range>(topic, 1, &UltrasonicProcessing::UltrasonicRawCb, this);
    uss_pub_ = nh_.advertise<sensor_msgs::Range>(ss_.str(), 1);
  }

  ~UltrasonicProcessing()
  {
    //nothing to do
  }

  void UltrasonicRawCb(const sensor_msgs::Range::ConstPtr &msg)
  {
    range_msg_preprocessed_uss_ = *msg;
    range_msg_preprocessed_uss_.range = filterRange(msg->range);
    checkForEmergencyStop(msg->range);
    uss_pub_.publish(range_msg_preprocessed_uss_);
  }

private:
  sensor_msgs::Range::_range_type filterRange(sensor_msgs::Range::_range_type range)
  {
    sensor_msgs::Range::_range_type ret_range = 0;
    if (range > rangeLimit)
    {
      ret_range = rangeLimit;
    }
    else
    {
      ret_range = range;
    }
    return ret_range;
  }

  void checkForEmergencyStop(float range)
  {
    emergancyStop_msg_preprocessed_uss_.data = false;
    if (0.3 > range)
    {
      emergancyStop_msg_preprocessed_uss_.data = true;
    }
    emergancyStop_pub_.publish(emergancyStop_msg_preprocessed_uss_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uss_prep");
  ros::NodeHandle nh;

  // front
  UltrasonicProcessing uss1_prep("uss_fl", nh);
  UltrasonicProcessing uss2_prep("uss_fml", nh);
  UltrasonicProcessing uss3_prep("uss_fmr", nh);
  UltrasonicProcessing uss4_prep("uss_fr", nh);
  // back
  UltrasonicProcessing uss5_prep("uss_bl", nh);
  UltrasonicProcessing uss6_prep("uss_bml", nh);
  UltrasonicProcessing uss7_prep("uss_bmr", nh);
  UltrasonicProcessing uss8_prep("uss_br", nh);

  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
