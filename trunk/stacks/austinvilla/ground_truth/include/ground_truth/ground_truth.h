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

namespace ground_truth {

  const int MAX_GROUND_POINTS = 5;  ///< /brief number of ground points in first step of calibration

  
  class Calibrator {
    public:

      Eigen::Vector3f rayPt1;  
      Eigen::Vector3f rayPt2;

      pcl::TransformationFromCorrespondences rigidBodyTransform;  ///< /brief transforms point cloud from kinect reference frame to the global one

      Eigen::Vector3f groundPoints[MAX_GROUND_POINTS];
      Eigen::Vector3f landmarkPoints[NUM_GROUND_PLANE_POINTS];
      bool landmarkAvailable[NUM_GROUND_PLANE_POINTS];






  };

}

#endif /* end of include guard: GROUND_TRUTH_RU3S44SI */
