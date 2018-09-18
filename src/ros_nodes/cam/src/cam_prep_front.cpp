#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp> // Basic OpenCV structures (cv::Mat, Scalar)
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

// blur parameters
int blurSize = 3;
// treshold parameters
double thresholdTresh = 240;
double thresholdMaxval = 255;
int thresholdType = THRESH_BINARY;
// canny edge parameters
int lowThreshold = 80;
int ratio = 3;
int kernel_size = 3;

Mat videoFrameGray;

class CameraPreprocessingFront
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  CameraPreprocessingFront()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cameraFront/image_raw", 1,
                               &CameraPreprocessingFront::imageCb, this);
    image_pub_ = it_.advertise("/cam_prep_front/output_video", 1);
  }

  ~CameraPreprocessingFront()
  {
    //nothing to do
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // image preprocessing
    // convert to grayscale
    cvtColor(cv_ptr->image, videoFrameGray, COLOR_RGBA2GRAY);
    // gaussian blur
    blur(videoFrameGray, videoFrameGray, Size(blurSize, blurSize));
    // treshold parameters
    threshold(videoFrameGray, videoFrameGray, thresholdTresh, thresholdMaxval, thresholdType);
    // Canny Edge detection
    Canny(videoFrameGray, videoFrameGray, lowThreshold, lowThreshold * ratio, kernel_size);

    sensor_msgs::ImagePtr ret_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", videoFrameGray).toImageMsg();

    // Output modified video stream
    image_pub_.publish(ret_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_prep_front");
  CameraPreprocessingFront cpf;
  ros::spin();
  return 0;
}
