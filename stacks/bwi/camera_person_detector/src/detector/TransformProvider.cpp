#include "TransformProvider.h"

TransformProvider::TransformProvider() {
}

TransformProvider::TransformProvider(std::string mapFrameID) {
  _groundPlaneAvailable = false;
  _mapFrameID = mapFrameID;
}

bool TransformProvider::isGroundPlaneAvailable() {
  return _groundPlaneAvailable;
}

void TransformProvider::computeModel(const sensor_msgs::CameraInfoConstPtr& cam_info) {
  _model.fromCameraInfo(cam_info);
}

void TransformProvider::computeGroundPlane(std::string camera_frame_id) {

  // Obtain transformation to camera
  tf::TransformListener listener;
  bool transform_found =
    listener.waitForTransform(camera_frame_id, _mapFrameID,
                              ros::Time(), ros::Duration(1.0));
  if (transform_found) {
    try {
      listener.lookupTransform(camera_frame_id, _mapFrameID,
                               ros::Time(), _tfCamFromMap);
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Transform unavailable (Exception): " << ex.what());
    }
  } else {
    ROS_ERROR_STREAM("Transform unavailable: lookup failed for frame " << camera_frame_id.c_str() << " from frame " << _mapFrameID.c_str());
  }

  _tfMapFromCam = _tfCamFromMap.inverse();

  tf::Point o_map(0,0,0);
  tf::Point p_map(1,0,0);
  tf::Point q_map(0,1,0);
  tf::Point ground_point;
  ground_point = _tfCamFromMap * o_map;
  tf::Point p_cam(_tfCamFromMap * p_map);
  tf::Point q_cam(_tfCamFromMap * q_map);

  _groundNormal = (p_cam - ground_point).cross(q_cam - ground_point);

  _groundPlaneAvailable = true;

}

double TransformProvider::getWorldHeight(cv::Point top, cv::Point bottom) {
  tf::Point camWorld = _tfMapFromCam * tf::Point(0,0,0);
  tf::Point groundWorld = getWorldProjection(top);
  tf::Point feetWorld = getWorldProjection(bottom);
  double d1 = groundWorld.distance(feetWorld);
  double d2 = groundWorld.distance(camWorld);
  double t = d1 / d2;
  // Use the x coordinate arbitrarily, any will do if the denominator is non-zero
  //double t = (groundWorld.x() - feetWorld.x()) / (groundWorld.x() - camWorld.x());
  tf::Point headWorld = t * (camWorld - groundWorld) + groundWorld;
  double height = headWorld.z() - feetWorld.z();
  return height;
}

tf::Point TransformProvider::getWorldProjection(cv::Point pt, float height) {

  cv::Point2d image_point(pt.x, pt.y);
  cv::Point2d rectified_point(_model.rectifyPoint(image_point));
  cv::Point3d ray = _model.projectPixelTo3dRay(rectified_point);

  tf::Point ray_1(0, 0, 0);
  tf::Point ray_2(ray.x, ray.y, ray.z);
  tf::Point ground_origin = _tfCamFromMap * tf::Point(0,0,height);
  float t = (ground_origin - ray_1).dot(_groundNormal)
          / (ray_2 - ray_1).dot(_groundNormal);
  tf::Point point_cam = ray_1 + t * (ray_2 - ray_1);
  return _tfMapFromCam * point_cam;

}

cv::Point TransformProvider::getImageProjection(tf::Point pt) {
  tf::Point point_cam = _tfCamFromMap * pt;
  cv::Point3d xyz(point_cam.x(), point_cam.y(), point_cam.z());
  cv::Point2d rectified_point(_model.project3dToPixel(xyz));
  return _model.unrectifyPoint(rectified_point);
}

