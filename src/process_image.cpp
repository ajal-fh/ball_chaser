#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "ball_chaser/DriveToTarget.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class BallChaser
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::ServiceClient client_;

public:
  BallChaser() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BallChaser::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    client_ = nh_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~BallChaser()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Hough circle detection
    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 16, 200, 30, 0, 0);
    if (circles.size() == 0)
      stop();
    for (size_t i = 0; i < circles.size(); i++)
    {
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      ROS_INFO("x: %d, y: %d", c[0], c[1]);
      move(c[0]);
      cv::circle(cv_ptr->image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle(cv_ptr->image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    //imshow("detected circles", cv_ptr->image);

    /*// Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);*/
    //cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void move(double x)
  {
    float linear, angular;
    if (x >= 0 && x <= 200)
    {
      linear = 0.5;
      angular = -0.5;
    }
    else if (x > 200 && x <= 600)
    {
      linear = 1.0;
      angular = 0;
    }
    else if (x > 600 && x <= 800)
    {
      linear = 0.5;
      angular = 0.5;
    }
    // call service client
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear;
    srv.request.angular_z = angular;
    if (client_.call(srv))
    {
      ROS_INFO("%s\n", srv.response.msg_feedback.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service ");
    }
  }
  void stop()
  {
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = 0;
    srv.request.angular_z = 0;
    if (client_.call(srv))
    {
      ROS_INFO("%s\n", srv.response.msg_feedback.c_str());
    }
    else
    {
      ROS_ERROR("Failed to call service ");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  BallChaser ic;
  ros::spin();
  return 0;
}
