#pragma once

#include "node.h"
#include <map>
#include <vector>
#include <iostream>

class AstarPathFinder {
 private:
  Eigen::MatrixXd data_;
  GridNodePtr **GridNodeMap_;
  GridNodePtr terminaterPtr_;
  Eigen::Vector2i start_coord_;
  Eigen::Vector2i goal_coord_;
  int map_size_x_;
  int map_size_y_;
  std::multimap<double, GridNodePtr> openSet_;
  std::vector<GridNodePtr> path_;

 private:
  double getHeu(const GridNodePtr& node_1, const GridNodePtr& node_2);
  void getNodeNeighbor(const GridNodePtr&        currentPtr,
                       std::vector<GridNodePtr>& neighborPtrSets,
                       std::vector<double>& edgeSets);
  bool isOccupied(const Eigen::Vector2i& coord);
  bool isFree(const Eigen::Vector2i& coord);

 public:
  AstarPathFinder();
  ~AstarPathFinder(){};
  void initGridMap(const Eigen::MatrixXd& map);
  void pathSearch(const Eigen::Vector2i& start_pt,
                  const Eigen::Vector2i& end_pt);
  void getPath();
};
