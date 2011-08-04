#ifndef __FIELD_H__
#define __FIELD_H__

#include <Eigen/Core>
#include <opencv/cv.h>
#include <pcl_visualization/pcl_visualizer.h>

namespace ground_truth {

  /* Constants defining the positions of various field points */

  const float FIELD_Y = 3.950; 
  const float FIELD_X = 5.950; 

  const float GRASS_Y = 4.725; 
  const float GRASS_X = 6.725; 

  const float PENALTY_Y = 2.150;  
  const float PENALTY_X =  0.550; 
  const float CIRCLE_RADIUS =  0.650; 

  const float PENALTY_CROSS_X = 1.200;

  const float GOAL_HEIGHT = 0.8;
  const float GOAL_Y = 1.5;

  /* Different points of interest on the field */

  /* Ground points that are easily identifiable */

  enum GroundPoints {
    YELLOW_BASE_TOP = 0,
    YELLOW_BASE_PENALTY_TOP = 1,
    YELLOW_GOALPOST_TOP = 2,
    YELLOW_GOALPOST_BOTTOM = 3,
    YELLOW_BASE_PENALTY_BOTTOM = 4,
    YELLOW_BASE_BOTTOM = 5,
    YELLOW_PENALTY_TOP = 6,
    YELLOW_PENALTY_BOTTOM = 7,
    YELLOW_PENALTY_CROSS = 8,
    MID_TOP = 9,
    MID_CIRCLE_TOP = 10,
    MID_CIRCLE_BOTTOM = 11,
    MID_BOTTOM = 12,
    BLUE_BASE_TOP = 13,
    BLUE_BASE_PENALTY_TOP = 14,
    BLUE_GOALPOST_TOP = 15,
    BLUE_GOALPOST_BOTTOM = 16,
    BLUE_BASE_PENALTY_BOTTOM = 17,
    BLUE_BASE_BOTTOM = 18,
    BLUE_PENALTY_TOP = 19,
    BLUE_PENALTY_BOTTOM = 20,
    BLUE_PENALTY_CROSS = 21,
    NUM_GROUND_PLANE_POINTS = 22
  };

  /* High points - top corners of goals */

  enum HighPoints {
    YELLOW_GOALPOST_TOP_HIGH = 0,
    YELLOW_GOALPOST_BOTTOM_HIGH = 1,
    BLUE_GOALPOST_TOP_HIGH = 2,
    BLUE_GOALPOST_BOTTOM_HIGH = 3,
    NUM_HIGH_POINTS = 4
  };

  class FieldProvider {

    private:

      Eigen::Vector3f centerField;
      Eigen::Vector3f groundPoints[NUM_GROUND_PLANE_POINTS];
      Eigen::Vector3f highPoints[NUM_HIGH_POINTS];

      /* Helper Functions */

      void draw2dLine(IplImage* image, const Eigen::Vector3f &ep1, const Eigen::Vector3f &ep2, const CvScalar &color, int width);
      void draw2dCircle(IplImage * image, const Eigen::Vector3f &pt, int radius, const CvScalar &color, int width);
      void draw2dCenterCircle(IplImage *image, const Eigen::Vector3f &centerPt, const Eigen::Vector3f &circlePt, const CvScalar &color, int width);
      void convertCoordinates(cv::Point2d &pos2d, int height, int width, const Eigen::Vector3f &pos3d);

      void draw3dLine(pcl_visualization::PCLVisualizer &visualizer, const Eigen::Vector3f &ep1, const Eigen::Vector3f &ep2, double r, double g, double b, const std::string &name);
      void draw3dCenterCircle(pcl_visualization::PCLVisualizer &visualizer, const Eigen::Vector3f &centerPt, const Eigen::Vector3f &circlePt, double r, double g, double b, const std::string &name);

    public:

      FieldProvider (float x = 0.0, float y = 0.0, float z = 0.0);
      void get2dField(IplImage* image, int highlightPoint = -1);
      void get3dField(pcl_visualization::PCLVisualizer &visualizer);
      inline Eigen::Vector3f getGroundPoint(int index) {
        return groundPoints[index];
      }

  };
}

#endif
