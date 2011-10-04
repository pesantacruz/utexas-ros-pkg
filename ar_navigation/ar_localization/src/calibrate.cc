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

#define NODE "ar_localization_calibrate"

namespace {

   

}

int main (int argc, char **argv) {

}
