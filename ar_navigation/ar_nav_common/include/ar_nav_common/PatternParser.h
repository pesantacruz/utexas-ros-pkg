/**
 * \file  PatternParser.h
 * \brief  Reads in patterns file from YAML Syntax
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 10/02/2011 12:49:43 PM piyushk $
 */

#ifndef PATTERNPARSER_KQJF84V6
#define PATTERNPARSER_KQJF84V6

namespace ar_nav {

  struct Vec3f {
    float x;
    float y;
    float z;
  };

  struct Pattern {
    std::string name;
    std::string patternFile;
    Vec3f location;
    float size;
  };

  void operator >> (const YAML::Node &node, Vec3f &v) {
     node[0] >> v.x;
     node[1] >> v.y;
     node[2] >> v.z;
  }

  void operator >> (const YAML::Node &node, Pattern &p) {
    node["name"] >> p.name;
    node["location"] >> p.location;
    node["patternFile"] >> p.patternFile;
    node["size"] >> p.size;
  }

  void operator >> (const YAML::Node &doc, std::vector<Pattern> &patterns) {
    patterns.clear();
    for (unsigned int i = 0; i < doc.size(); i++) {
      Pattern p;
      doc[i] >> p;
      patterns.push_back(p);
    }
  }

  void readPatternFile(std::string fileName, std::vector

}

#endif /* end of include guard: PATTERNPARSER_KQJF84V6 */
