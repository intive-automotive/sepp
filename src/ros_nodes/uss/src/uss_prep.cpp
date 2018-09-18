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
