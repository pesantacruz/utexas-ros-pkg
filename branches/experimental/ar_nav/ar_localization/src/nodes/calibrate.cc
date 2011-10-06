/**
 * \file  calibrate.cc
 * \brief  Calculates position and orientation of any cameras using
 *         ARTags placed in known locations
 *
 * The ar_multi node is laucnched automatically using roslaunch, and the
 * configuration is provided using a YAML file. ar_multi should already be
 * publishing this information on tf.
 *
 * A static transformation publisher should then be publishing the transform
 * between marker[i] -> world[i] (since the parent of the tree structure 
 * will be base_link on the camera, we require multiple worlds here)
 *
 * In this node we read the yaml file to figure out the number of markers
 * available, and read the corresponding transformations available from the
 * tf tree (i.e. world[i] -> camera_base_link).
 *
 * We average accross all of these to reduce error, and write out the
 * /world -> /camera_base_link transformation to file. The camera is
 * calibrated!
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 09/30/2011 11:25:52 AM piyushk $
 */


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/package.h>

#include <boost/foreach.hpp>

#include <ar_nav_common/PatternParser.h>

#define NODE "ar_localization_calibrate"

namespace {
  std::string patternFile;
  std::string cameraBaseFrame;
  std::string cameraId;
}

void getParams(ros::NodeHandle& nh) {
  nh.getParam("patternFile", patternFile);
  nh.getParam("cameraBaseFrame", cameraBaseFrame);
  nh.getParam("cameraId", cameraId);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle nh;
  getParams(nh);

  // Get information about the patterns
  std::vector<ar_nav::Pattern> patterns;
  ar_nav::readPatternFile(patternFile, patterns);

  tf::TransformListener listener(nh);
  tf::Vector3 averageTranslation(0,0,0);
  tf::Quaternion averageRotation(0,0,0,0);
  int numTransformations = 0;

  // Get transformations for every marker
  BOOST_FOREACH(const ar_nav::Pattern& pattern, patterns) {

    // Get transform from /marker to /camera
    tf::StampedTransform transformMarkerToCam;
    bool transformAvailable = true;
    try {
      listener.lookupTransform(cameraBaseFrame, pattern.name,  
                               ros::Time(0), transformMarkerToCam);
    } catch (tf::TransformException ex) {
      transformAvailable = false;
      ROS_ERROR("Error getting transformation for %s: %s", pattern.name.c_str(), ex.what());
    }

    if (!transformAvailable) {
      continue;
    } 

    // Compute transform of /map to /marker
    tf::Quaternion noRotation(0, 0, 0, 0);
    tf::Vector3 markerPosition(pattern.location.x, pattern.location.y, pattern.location.z);
    tf::Transform transformWorldToMarker(noRotation, markerPosition);

    // Compute transformation from /map to /camera
    tf::Transform transformWorldToCam(tf::Quaternion(0,0,0,0), tf::Vector3(0,0,0));
    transformWorldToCam.mult(transformWorldToMarker, transformWorldToCam);

    averageTranslation += transformWorldToCam.getOrigin();
    averageRotation += transformWorldToCam.getRotation();  
    numTransformations++;  

  }

  if (numTransformations == 0) {
    ROS_ERROR("Unable to get any viable transforms from markers");
    return 0;
  }

  averageTranslation *= 1.0 / numTransformations;
  averageRotation *= 1.0 / numTransformations;

  std::ofstream fout((ros::package::getPath("ar_localization") + "/launch/" + cameraId).c_str());

  fout << "<launch>" << std::endl;
  fout << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"link_" << cameraId << "_broadcaster\""
       << "args=\"" << averageTranslation.x() << " " << averageTranslation.y() << " " << averageTranslation.z()
       << " " << averageRotation.getX() << " " << averageRotation.getY() << " " << averageRotation.getZ() << " "
       << averageRotation.getW() << " map " << cameraBaseFrame << " 100\" />" << std::endl;
  fout << "</launch>" << std::endl;
  fout.close();

  return 0;
}
