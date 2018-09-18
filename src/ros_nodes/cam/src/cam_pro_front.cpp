#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat, Scalar)

using namespace std;
using namespace cv;

// hough line parameters
int houghRho = 1;
double houghTheta = CV_PI / 180;
int houghTresshold = 60;
int houghMinLinLength = 5;
int houghMaxLineGap = 350;

vector<Vec4i> lines;

Mat videoFrameGray;

class CameraProcessingFront
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  CameraProcessingFront()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam_prep_front/output_video", 1,
                               &CameraProcessingFront::imageCb, this);
    image_pub_ = it_.advertise("/cam_pro_front/output_video", 1);
  }

  ~CameraProcessingFront()
  {
    //nothing to do
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    HoughLinesP(cv_ptr->image, lines, houghRho, houghTheta, houghTresshold, houghMinLinLength, houghMaxLineGap);
    for (size_t i = 0; i < lines.size(); i++)
    {
      Vec4i l = lines[i];
      line(cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }

    // Output modified video stream

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_pro_front");
  CameraProcessingFront cpf;
  ros::spin();
  return 0;
}
