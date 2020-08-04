#pragma once

#include <eigen3/Eigen/Core>
#include <cfloat>
#include <map>

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode {
  int id;                 // 1 --> open set, -1 --> closed set
  Eigen::Vector2i coord;
  double gScore;
  double hScore;
  double fScore;
  GridNodePtr cameFrom;
//  std::multimap<double, GridNodePtr>::iterator nodeMapIt;

  GridNode(Eigen::Vector2i coord_) {
    id       = 0;
    coord    = coord_;
    gScore   = DBL_MAX;
    hScore   = DBL_MAX;
    fScore   = gScore + hScore;
    cameFrom = NULL;
  }

};


