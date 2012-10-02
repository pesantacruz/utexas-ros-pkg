#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <april_msgs/TagPoseArray.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

namespace {
  cv_bridge::CvImageConstPtr imageMsgPtr; 
  bool first_image_received = false;
}

void drawDetections(cv_bridge::CvImageConstPtr& input, 
    const april_msgs::TagPoseArray::ConstPtr& detections,
    cv::Mat& output) {
  output = input->image.clone();
  BOOST_FOREACH(const april_msgs::TagPose& tag, detections->tags) {
    int length = 4;
    for (int i = 0; i < length; i++) {
      cv::Point p1(tag.image_coordinates[i].x, tag.image_coordinates[i].y);
      cv::Point p2(tag.image_coordinates[(i + 1) % length].x, 
          tag.image_coordinates[(i + 1) % length].y);
      cv::line(output, p1, p2, cv::Scalar(255,0,0));
    }
  }
}


void processDetections(const april_msgs::TagPoseArray::ConstPtr& detections) {
  if (first_image_received) {
    cv::Mat output;
    drawDetections(imageMsgPtr, detections, output);
    cv::imshow("Output", output);
  }
}

void processImage(const sensor_msgs::ImageConstPtr &msg) {
  imageMsgPtr = cv_bridge::toCvShare(msg, "bgr8");
  first_image_received = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "april_test");

  cv::namedWindow("Output");
  cvResizeWindow("Output", 320, 240);
  cvStartWindowThread();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tags", 1, processDetections);

  image_transport::ImageTransport it(n);
  std::string image_topic = n.resolveName("usb_cam/image_raw");
  image_transport::Subscriber center_camera =	it.subscribe(image_topic, 1, &processImage);

  ros::spin();
  return 0;
}
