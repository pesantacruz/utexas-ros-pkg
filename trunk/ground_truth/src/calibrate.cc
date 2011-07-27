// ROS + core
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <terminal_tools/parse.h>

// OpenCV + Highgui for image display
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <Eigen/Core>

#include <ground_truth/field.h>

// Different states that the calibrator can be in
#define COLLECT_GROUND_POINTS 0
#define GET_GROUND_POINT_INFO 1

#define COLLECT_LANDMARKS 5
#define GET_LANDMARK_INFO 6

#define MAX_GROUND_POINTS 5
#define MAX_GET_INFO_ATTEMPTS 5

#define RGB_IMAGE_WIDTH 640
#define RGB_IMAGE_HEIGHT 480

#define SELECTOR_IMAGE_WIDTH 240
#define SELECTOR_IMAGE_HEIGHT 180

using terminal_tools::parse_argument;

namespace {

  sensor_msgs::PointCloud2ConstPtr cloudPtr, oldCloudPtr;
  pcl::PointCloud<pcl::PointXYZRGB> cloud, transformedCloud;
  boost::mutex mCloud;
  pcl_visualization::PointCloudColorHandler<pcl::PointXYZRGB>::Ptr colorHandler;
  pcl_visualization::PointCloudGeometryHandler<pcl::PointXYZRGB>::Ptr geometryHandler;

  IplImage* rgbImage;
  boost::mutex mImage;
  sensor_msgs::CvBridge bridge;
  image_geometry::PinholeCameraModel model;

  Eigen::Vector3f rayPt1, rayPt2;
  pcl::PointXYZ sphere;

  IplImage * selectorImage;
  pcl::TransformationFromCorrespondences rigidBodyTransform;
  Eigen::Vector3f pc_point;

  int state = COLLECT_GROUND_POINTS;

  Eigen::Vector3f groundPoints[MAX_GROUND_POINTS];
  int numGroundPoints = 0;
  int displayGroundPoints = 0;

  Eigen::Vector3f landmarkPoints[ground_truth::NUM_GROUND_PLANE_POINTS];
  bool landmarkAvailable[ground_truth::NUM_GROUND_PLANE_POINTS];
  int currentLandmark = 0;
  int displayLandmark = 0;

  bool transformationAvailable = false;
  Eigen::Affine3f transformMatrix;

  Eigen::Vector4f groundPlaneParameters;

  pcl_visualization::PCLVisualizer *visualizerPtr;

  bool newDisplayLandmark;
  ground_truth::Field fieldProvider;

  std::string calibFile = "data/calib.txt";

}

inline std::string getUniqueName(const std::string &baseName, int uniqueId) {
  return baseName + boost::lexical_cast<std::string>(uniqueId);
}

template <typename T>
void noDelete(T *ptr) {
}

void displayCloud(pcl_visualization::PCLVisualizer &visualizer, pcl::PointCloud<pcl::PointXYZRGB> &cloudToDisplay) {
  
  pcl::PointCloud<pcl::PointXYZRGB> displayCloud;

  /* This Filter code is currently there, due to some failure for nan points while displaying */

  //Filter to remove NaN points
  pcl::PointIndices inliers;
  for (int i = 0; i < cloudToDisplay.points.size(); i++) {
    pcl::PointXYZRGB *pt = &cloud.points[i];
    if (pcl_isfinite(pt->x))
      inliers.indices.push_back(i);
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ptr(&cloudToDisplay, noDelete<pcl::PointCloud<pcl::PointXYZRGB> >);
  extract.setInputCloud(ptr);
  extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
  extract.setNegative(false);
  extract.filter(displayCloud);

  visualizer.removePointCloud();
  colorHandler.reset (new pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (displayCloud));
  geometryHandler.reset (new pcl_visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZRGB> (displayCloud));
  visualizer.addPointCloud<pcl::PointXYZRGB>(displayCloud, *colorHandler, *geometryHandler);
}

void calculateTransformation() {
  for (int i = 0; i < ground_truth::NUM_GROUND_PLANE_POINTS; i++) {
    if (landmarkAvailable[i]) {
      rigidBodyTransform.add(landmarkPoints[i], fieldProvider.getGroundPoint(i), 1.0 / (landmarkPoints[i].norm() * landmarkPoints[i].norm()));
    }
  }
  transformMatrix = rigidBodyTransform.getTransformation();
  transformationAvailable = true;

  // Output to file
  std::ofstream fout(calibFile.c_str());
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      fout << transformMatrix(i,j) << " ";
    }
    fout << endl;
  }
  fout.close();

}

void calculateGroundPlane() {
  pcl::PointCloud<pcl::PointXYZ> groundPlaneCloud;
  pcl::PointIndices inliers;
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimator;

  for (unsigned int i = 0; i < MAX_GROUND_POINTS; i++) {
    pcl::PointXYZ point;
    point.x = groundPoints[i].x();
    point.y = groundPoints[i].y();
    point.z = groundPoints[i].z();
    groundPlaneCloud.points.push_back(point);
    inliers.indices.push_back(i);
  }

  float curvature;
  normalEstimator.computePointNormal(groundPlaneCloud, inliers.indices, groundPlaneParameters, curvature);
  //std::cout << "Ground Plane Parameters: " << std::endl << groundPlaneParameters << std::endl;
}

Eigen::Vector3f getPointFromGroundPlane() {
  
  //Obtain a point and normal for the plane
  Eigen::Vector3f c(0,0,-groundPlaneParameters(3)/groundPlaneParameters(2));
  Eigen::Vector3f n(groundPlaneParameters(0), groundPlaneParameters(1), groundPlaneParameters(2));

  float t = (c - rayPt1).dot(n) / (rayPt2 - rayPt1).dot(n);
  
  Eigen::Vector3f point;  
  point = rayPt1 + t*(rayPt2 - rayPt1);

  return point;
}

void displayStatus(const std::string &status) {
  //ROS_INFO(status.c_str());
  std::cout << status << endl;
  visualizerPtr->removeShape("status");
  visualizerPtr->addText(status,320,0,"status");
}

void collectRayInfo(int x, int y) {
  cv::Point2d origPt(x, y), rectPt;
  model.rectifyPoint(origPt, rectPt);
  std::cout << origPt << std::endl << rectPt;
  cv::Point3d ray;
  model.projectPixelTo3dRay(rectPt, ray);
  rayPt1 = Eigen::Vector3f(0,0,0);
  rayPt2 = Eigen::Vector3f(ray.x, ray.y, ray.z);
}

void imageMouseCallback(int event, int x, int y, int flags, void* param) {

  switch(event) {

    case CV_EVENT_LBUTTONDOWN: {

      switch(state) {
        case COLLECT_GROUND_POINTS: {             // Obtain ground point (lclick)
          collectRayInfo(x, y);
          state = GET_GROUND_POINT_INFO;
          break;
        }
        case COLLECT_LANDMARKS: {
          if (flags & CV_EVENT_FLAG_CTRLKEY) {    // Deselect Landmark (ctrl + lclick)
            landmarkAvailable[currentLandmark] = false;
            newDisplayLandmark = false;
          } else {                                // Obtain current landmark (lclick)
            collectRayInfo(x, y);
            state = GET_LANDMARK_INFO;
          }
          break;
        }
      }

      break; // outer
    }

    case CV_EVENT_RBUTTONDOWN: {

      switch(state) {
        case COLLECT_GROUND_POINTS: {
          numGroundPoints--;
          char s[100];
          sprintf(s, "Select Ground Point (%i of %i)", numGroundPoints+1, MAX_GROUND_POINTS);
          displayStatus(s);
          break;
        }
        case COLLECT_LANDMARKS: {
          if (flags & CV_EVENT_FLAG_CTRLKEY) {    // Go back to previous landmark (ctrl + rclick)
            currentLandmark--;
            currentLandmark = (currentLandmark < 0) ? 0 : currentLandmark;
            fieldProvider.get2dField(selectorImage, currentLandmark);
            cvShowImage("Selector", selectorImage);
            ROS_INFO("Select Landmark (%i of %i)", currentLandmark+1, ground_truth::NUM_GROUND_PLANE_POINTS);
          } else {                                // Go to next landmark (rclick)
            currentLandmark++;
            if (currentLandmark == ground_truth::NUM_GROUND_PLANE_POINTS) {
              calculateTransformation();
              currentLandmark = ground_truth::NUM_GROUND_PLANE_POINTS - 1;
            } else {
              fieldProvider.get2dField(selectorImage, currentLandmark);
              cvShowImage("Selector", selectorImage);
              ROS_INFO("Select Landmark (%i of %i)", currentLandmark+1, ground_truth::NUM_GROUND_PLANE_POINTS);
            }
          }
          break;
        }

      }
    
      break; // outer
    }
  }


}

void imageCallback(const sensor_msgs::ImageConstPtr& image,
    const sensor_msgs::CameraInfoConstPtr& camInfo) {
  ROS_DEBUG("Image received height %i, width %i", model.height(), model.width());
  mImage.lock();
  rgbImage = bridge.imgMsgToCv(image, "bgr8");
  model.fromCameraInfo(camInfo);
  mImage.unlock();
}

/* Distance of a point from a ray */
float distanceLineFromPoint(Eigen::Vector3f ep1, Eigen::Vector3f ep2, Eigen::Vector3f point) {
  return ((point - ep1).cross(point - ep2)).norm() / (ep2 - ep1).norm();
}

Eigen::Vector3f getPointFromCloud() {
  
  unsigned int count = 0;
  Eigen::Vector3f averagePt(0, 0, 0);

  for (unsigned int i = 0; i < cloud.points.size(); i++) {

    pcl::PointXYZRGB *pt = &cloud.points[i];

    // Failed Points
    if (!pcl_isfinite(pt->x))
      continue;

    // Calculate Distance for valid points
    Eigen::Vector3f point(pt->x, pt->y, pt->z); 
    float distance = distanceLineFromPoint(rayPt1, rayPt2, point);
    if (distance < 0.025) { // within 2.5 cm of the ray
      averagePt+= point;
      count++;
    }
  }
  averagePt /= count;

  return averagePt;
}

void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloudPtrMsg) {
  ROS_DEBUG("PointCloud with %d, %d data points (%s), stamp %f, and frame %s.", cloudPtrMsg->width, cloudPtrMsg->height, pcl::getFieldsList (*cloudPtrMsg).c_str (), cloudPtrMsg->header.stamp.toSec (), cloudPtrMsg->header.frame_id.c_str ()); 
  mCloud.lock();
  cloudPtr = cloudPtrMsg;
  mCloud.unlock();
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "kinect_position_calibrator");
  ros::NodeHandle nh;

  // Get the queue size from the command line
  int queue_size = 1;
  parse_argument (argc, argv, "-qsize", queue_size);

  nh.param("calibFile", calibFile, calibFile);
  terminal_tools::parse_argument (argc, argv, "-calibFile", calibFile);
  ROS_INFO("Calib File: %s", calibFile.c_str());

  // Create a ROS subscriber for the point cloud
  ros::Subscriber subCloud = nh.subscribe ("input", queue_size, cloudCallback);

  // Subscribe to image using image transport
  image_transport::ImageTransport it(nh);
  image_transport::CameraSubscriber subImage = it.subscribeCamera("inputImage", 1, imageCallback);

  // Stuff to display the point cloud properly
  pcl_visualization::PCLVisualizer visualizer (argc, argv, "Online PointCloud2 Viewer");
  visualizer.addCoordinateSystem(); // Good for reference
  visualizerPtr = &visualizer;

  // Stuff to display the rgb image
  cvStartWindowThread();
  cvNamedWindow("ImageCam");
  cvMoveWindow("ImageCam", 0,0);
  rgbImage = cvCreateImage(cvSize(RGB_IMAGE_WIDTH, RGB_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
  cvSetMouseCallback( "ImageCam", imageMouseCallback);

  // Stuff to display the selector
  cvNamedWindow("Selector");
  cvMoveWindow("Selector", 10, 700);
  currentLandmark = 0;
  selectorImage = cvCreateImage(cvSize(SELECTOR_IMAGE_WIDTH, SELECTOR_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
  fieldProvider.get2dField(selectorImage, currentLandmark);
  cvShowImage("Selector", selectorImage);
  
  char s[100];
  sprintf(s, "Select Ground Point (%i of %i)", numGroundPoints+1, MAX_GROUND_POINTS);
  displayStatus(s);

  while (nh.ok ()) {
    // Spin
    ros::spinOnce ();
    ros::Duration (0.001).sleep ();
    visualizer.spinOnce (10);

    // If no cloud received yet, continue
    if (!cloudPtr)
      continue;

    if (cloudPtr == oldCloudPtr)
      continue;

    mCloud.lock ();
    pcl::fromROSMsg(*cloudPtr, cloud);

    switch (state) {

      case GET_GROUND_POINT_INFO: {
        groundPoints[numGroundPoints] = getPointFromCloud();
        numGroundPoints++;
        if (numGroundPoints == MAX_GROUND_POINTS) {
          calculateGroundPlane();
          ROS_INFO("Select Landmark (%i of %i)", currentLandmark+1, ground_truth::NUM_GROUND_PLANE_POINTS);
          state = COLLECT_LANDMARKS;
          numGroundPoints = 0;
        } else {
          char s[100];
          sprintf(s, "Select Ground Point (%i of %i)", numGroundPoints+1, MAX_GROUND_POINTS);
          displayStatus(s);
          state = COLLECT_GROUND_POINTS; 
        }
        break;
      }

      case GET_LANDMARK_INFO: {
        landmarkPoints[currentLandmark] = getPointFromGroundPlane();
        //std::cout << "Point Selected: " << std::endl << landmarkPoints[currentLandmark] << std::endl;
        landmarkAvailable[currentLandmark] = true;
        state = COLLECT_LANDMARKS;
        ROS_INFO("Landmark Info obtained (%i of %i)", currentLandmark+1, ground_truth::NUM_GROUND_PLANE_POINTS);
        newDisplayLandmark = true;
        break;
      }
    }

    // Display point cloud
    if (transformationAvailable) {
      pcl::transformPointCloud(cloud, transformedCloud, transformMatrix);
      displayCloud(visualizer, transformedCloud);
    } else {
      displayCloud(visualizer, cloud);
    }

    // Display spheres during ground point selection
    if (displayGroundPoints != numGroundPoints) {
      // Add necessary spheres
      for (; displayGroundPoints < numGroundPoints; displayGroundPoints++) {
        pcl::PointXYZ point(groundPoints[displayGroundPoints].x(), groundPoints[displayGroundPoints].y(), groundPoints[displayGroundPoints].z());
        visualizer.addSphere(point, 0.05, 0,1,0, getUniqueName("ground", displayGroundPoints));
      }
      // Remove unnecessary spheres
      for (; displayGroundPoints > numGroundPoints; displayGroundPoints--) {
        visualizer.removeShape(getUniqueName("ground", displayGroundPoints - 1));
      }
    }

    // Display spheres durink landmaark
    // Remove
    if (displayLandmark != currentLandmark || newDisplayLandmark) {
      visualizer.removeShape(getUniqueName("landmark", displayLandmark));
      displayLandmark = currentLandmark;
    }
    if (displayLandmark == currentLandmark && landmarkAvailable[displayLandmark] && newDisplayLandmark) {
      pcl::PointXYZ point(landmarkPoints[displayLandmark].x(), landmarkPoints[displayLandmark].y(), landmarkPoints[displayLandmark].z());
      visualizer.addSphere(point, 0.05, 0,1,0, getUniqueName("landmark", displayLandmark));
      newDisplayLandmark = false;
    }

    // Use old pointer to prevent redundant display
    oldCloudPtr = cloudPtr;
    mCloud.unlock ();

    mImage.lock();
    cvShowImage("ImageCam", rgbImage);
    mImage.unlock();
  }

  return (0);
}
