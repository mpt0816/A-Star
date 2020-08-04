#include "Astar_search.h"

AstarPathFinder::AstarPathFinder() {
  terminaterPtr_ = NULL;
  map_size_x_    = INT_MAX;
  map_size_y_    = INT_MAX;
}

/** map -> Eigen::Matrix
 *  -----------> y (col)
 *  |
 *  |
 *  |
 *  x (row)
 */
void AstarPathFinder::initGridMap(const Eigen::MatrixXd &map) {
  data_.resize(map.rows(), map.cols());
  data_       = map;
  map_size_x_ = data_.rows();
  map_size_y_ = data_.cols();
  GridNodeMap_ = new GridNodePtr *[map_size_x_];
  for (int i = 0; i < map_size_x_; ++i) {
    GridNodeMap_[i] = new GridNodePtr [map_size_y_];
    for (int j = 0; j < map_size_y_; ++j) {
      GridNodeMap_[i][j] = new GridNode(Eigen::Vector2i(i, j));
    }
  }
}

double AstarPathFinder::getHeu(const GridNodePtr &node_1,
                               const GridNodePtr &node_2) {
  double hScore = 0.0;
  // Manhattan
  hScore = std::fabs(node_1->coord[0] - node_2->coord[0]) +
           std::fabs(node_1->coord[1] - node_2->coord[1]);
//  // Euclidean
//  hScore = std::sqrt(std::pow(node_1->coord[0] - node_2->coord[0], 2) +
//                     std::pow(node_1->coord[1] - node_2->coord[1], 2));
//  // Dijkstra
//  hScore = 0.0;
  return hScore;
}

void AstarPathFinder::getNodeNeighbor(const GridNodePtr& currentPtr,
                                      std::vector<GridNodePtr>& neighborPtrSets,
                                      std::vector<double>& edgeSets) {
  neighborPtrSets.clear();
  edgeSets.clear();
  Eigen::Vector2i curr_coord = currentPtr->coord;
  int count = 0;
  for (int i = std::max(curr_coord[0] - 1 + 0.5, 0.5);
           i < std::min(curr_coord[0] + 1 + 0.5, map_size_x_ + 0.5); ++i) {
    for (int j = std::max(curr_coord[1] - 1 + 0.5, 0.5);
             j < std::min(curr_coord[1] + 1 + 0.5, map_size_y_ + 0.5); ++j) {
      Eigen::Vector2i coord(i, j);
      if (coord == curr_coord ||
          isOccupied(coord)) {
        count += 1;
        continue;
      } else {
        GridNodePtr neighborPtr;
        neighborPtr = GridNodeMap_[i][j];
        neighborPtrSets.push_back(neighborPtr);
        double edge = std::sqrt(std::pow(curr_coord[0] - i, 2) +
            std::pow(curr_coord[1] - j, 2));
        edgeSets.push_back(edge);
      }
    }
  }
}

bool AstarPathFinder::isOccupied(const Eigen::Vector2i &coord) {
  if (coord[0] < 0 || coord[0] > map_size_x_ - 1 ||
      coord[1] < 0 || coord[1] > map_size_y_ - 1) {
    return true;
  }
  if (data_(coord[0], coord[1]) > 0.0) {
    return true;
  } else {
    return false;
  }
}

bool AstarPathFinder::isFree(const Eigen::Vector2i &coord) {
  return !isOccupied(coord);
}

void AstarPathFinder::pathSearch(const Eigen::Vector2i &start_pt,
                                 const Eigen::Vector2i &end_pt) {
  start_coord_ = start_pt;
  goal_coord_  = end_pt;
  openSet_.clear();

  GridNodePtr startPtr = new GridNode(start_coord_);
  GridNodePtr endPtr   = new GridNode(goal_coord_);
  startPtr->gScore = 0.0;
  startPtr->hScore = getHeu(startPtr, endPtr);
  startPtr->fScore = startPtr->gScore + startPtr->hScore;
  startPtr->id     = 1;
  openSet_.insert(std::make_pair<double, GridNodePtr>(startPtr->fScore, startPtr));
  GridNodePtr currentPtr  = NULL;
  GridNodePtr neighborPtr = NULL;

  std::vector<GridNodePtr> neighborPtrSets;
  std::vector<double> edgeSets;
  while (!openSet_.empty()) {
    std::multimap<double, GridNodePtr>::iterator itr_begin = openSet_.begin();
    currentPtr = openSet_.begin()->second;
    currentPtr->id = -1;
    if (currentPtr->coord == goal_coord_) {
      terminaterPtr_ = currentPtr;
      return;
    }
    getNodeNeighbor(currentPtr, neighborPtrSets, edgeSets);
    if (!neighborPtrSets.empty()) {
      for (int i = 0; i < neighborPtrSets.size(); ++i) {
        neighborPtr = neighborPtrSets[i];
        if (neighborPtr->id == 0) {
          neighborPtr->id = 1;
          neighborPtr->gScore = currentPtr->gScore + edgeSets[i];
          neighborPtr->hScore = getHeu(neighborPtr, endPtr);
          neighborPtr->fScore = neighborPtr->gScore + neighborPtr->hScore;
          neighborPtr->cameFrom = currentPtr;
          openSet_.insert(std::make_pair<double, GridNodePtr>(neighborPtr->fScore, neighborPtr));
        } else if (neighborPtr->id == 1) {
          neighborPtr->id = -1;
          double gScore_new = currentPtr->gScore + edgeSets[i];
          double fScore_new = gScore_new + neighborPtr->hScore;
          if (fScore_new < neighborPtr->fScore) {
            neighborPtr->gScore = gScore_new;
            neighborPtr->fScore = fScore_new;
            neighborPtr->cameFrom = currentPtr;
            openSet_.insert(std::make_pair<double, GridNodePtr>(neighborPtr->fScore, neighborPtr));
          }
        } else if (neighborPtr->id == -1){
          for (std::multimap<double, GridNodePtr>::iterator itr = openSet_.begin(); itr != openSet_.end(); ++itr) {
            if (itr->first  == neighborPtr->fScore &&
                itr->second == neighborPtr) {
              openSet_.erase(itr);
            }
          }
        }
      }
    }
    openSet_.erase(itr_begin);
  }
}

void AstarPathFinder::getPath() {
  if (terminaterPtr_ == NULL) {
    std::cout << "Not Find Goal Point." << std::endl;
    return;
  }
  path_.clear();
  while (terminaterPtr_ != NULL) {
    path_.push_back(terminaterPtr_);
    terminaterPtr_ = terminaterPtr_->cameFrom;
  }
  std::reverse(path_.begin(), path_.end());
  std::cout << "A-star Path is: " << std::endl;
  for (int i = 0; i < path_.size() - 1; ++i) {
    std::cout << "(" << path_[i]->coord[0] << ", " << path_[i]->coord[1] << ")" << " --> " << std::endl;
  }
  std::cout << "(" << path_[path_.size() - 1]->coord[0] << ", " << path_[path_.size() - 1]->coord[1] << ")" << std::endl;
}

