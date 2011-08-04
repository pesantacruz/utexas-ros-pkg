/**
 * \file  ground_truth.h
 * \brief Library for calibrating and using the system
 *
 * This library includes the core functionalities for the ground truth 
 * detection system and its calibration.
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 08/04/2011 02:47:36 PM piyushk $
 */

#ifndef GROUND_TRUTH_RU3S44SI
#define GROUND_TRUTH_RU3S44SI

#include "ground_truth/field_provider.h"

namespace ground_truth {

  const int MAX_GROUND_POINTS = 5;  ///< number of ground points in first step of calibration
  
  class Calibrator {
    public:

    Eigen::Vector3f rayPt1;  
    Eigen::Vector3f rayPt2;

    pcl::TransformationFromCorrespondences rigidBodyTransform;  ///< transforms point cloud from kinect reference frame to the global one

    Eigen::Vector3f groundPoints[MAX_GROUND_POINTS];
    int currentGroundPoint;
    
    Eigen::Vector3f landmarkPoints[NUM_GROUND_PLANE_POINTS];
    bool landmarkAvailable[NUM_GROUND_PLANE_POINTS];

    Eigen::Affine3f transformMatrix;
    Eigen::Vector4f groundPlaneParameters;

    /**
     * \brief   Calculates transformation based on recorded landmarks
     * \param   fieldProvider Provides the true locations for the landmarks
     * \return  The transformation matrix
     */
    Eigen::Affine3f calculateTransformation(const FieldProvider &fieldProvider) {
      for (int i = 0; i < NUM_GROUND_PLANE_POINTS; i++) {
        if (landmarkAvailable[i]) {
          rigidBodyTransform.add(landmarkPoints[i], fieldProvider.getGroundPoint(i), 1.0 / (landmarkPoints[i].norm() * landmarkPoints[i].norm()));
        }
      }
      transformMatrix = rigidBodyTransform.getTransformation();

      return transformMatrix;
    }

    /**
     * \brief  Calculates the ground plane from recorded points on the ground plane 
     */
    void calculateGroundPlane() {
      pcl::PointCloud<pcl::PointXYZ> groundPlaneCloud;
      pcl::PointIndices inliers;
      pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimator;

      for (unsigned int i = 0; i < MAX_GROUND_POINTS; i++) {
        pcl::PointXYZ point(groundPoints[i].x(), groundPoints[i].y(), groundPoints[i].z());
        groundPlaneCloud.points.push_back(point);
        inliers.indices.push_back(i);
      }

      float curvature;
      normalEstimator.computePointNormal(groundPlaneCloud, inliers.indices, groundPlaneParameters, curvature);
    }
    
    /**
     * \brief  Gets a point on the ground plane by passing a ray through the plane 
     * \return The point position 
     */
    Eigen::Vector3f getPointFromGroundPlane() {
      
      //Obtain a point and normal for the plane
      Eigen::Vector3f c(0,0,-groundPlaneParameters(3)/groundPlaneParameters(2));
      Eigen::Vector3f n(groundPlaneParameters(0), groundPlaneParameters(1), groundPlaneParameters(2));

      float t = (c - rayPt1).dot(n) / (rayPt2 - rayPt1).dot(n);
      
      Eigen::Vector3f point;  
      point = rayPt1 + t*(rayPt2 - rayPt1);

      return point;
    }

    /**
     * \brief Updates the position of the current ray selected by the user  
     * \param pt1 The first pt defining the ray (typically 0,0,0) 
     * \param pt2 The second pt defining the ray
     */
    void updateRay(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2) {
      rayPt1 = pt1;
      rayPt2 = pt2;
    }

  };

}

#endif /* end of include guard: GROUND_TRUTH_RU3S44SI */
