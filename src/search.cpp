#include "search.h"
#include <limits>
#include <cmath>
#include <iostream>

namespace astar {

AStar::AStar() {
  terminaterPtr_ = nullptr;
  map_max_x_ = std::numeric_limits<int>::epsilon();
  map_max_y_ = std::numeric_limits<int>::epsilon();
}

void AStar::InitGridMap(const Eigen::MatrixXd& map) {
  map_max_x_ = map.rows() - 1;
  map_max_y_ = map.cols() - 1;
  gridmap_ = new (double* [map.rows()]);
  for (int i = 0; i < map.rows(); ++i) {
    gridmap_[i] = new (double [map.cols()]);
  }
  for (int i = 0; i <= map_max_x_; ++i) {
    for (int j = 0; j <= map_max_y_; ++j) {
      gridmap_[i][j] = map(i, j);
    }
  }
}

void AStar::PathSearch(
    const Eigen::Vector2i& start_pt,
    const Eigen::Vector2i& end_pt) {
  NodePtr start = new Node(start_pt[0], start_pt[1]);
  NodePtr goal = new Node(end_pt[0], end_pt[1]);
  start->SethScore(ComputeHScore(start, goal));
  open_pq_.emplace(start);
  open_map_.emplace(start->GetIndex(), start);
  while (!open_pq_.empty()) {
    NodePtr current_node = open_pq_.top();
    open_pq_.pop();
    close_map_.emplace(current_node->GetIndex(), current_node);
    if (current_node->GetIndex() == goal->GetIndex()) {
      terminaterPtr_ = current_node;
      break;
    }
    std::vector<NodePtr> next_nodes = GetNodeNeighbor(current_node);
    for (auto& next_node : next_nodes) {
      if (close_map_.find(next_node->GetIndex()) != close_map_.end()) {
        continue;
      }
      next_node->SethScore(ComputeHScore(next_node, goal));
      next_node->SetgScore(ComputeGScore(current_node, next_node));
      auto itr = open_map_.find(next_node->GetIndex());
      if (itr == open_map_.end()) {
        open_pq_.emplace(next_node);
        open_map_.emplace(next_node->GetIndex(), next_node);
      } else {
        if (itr->second->GetfScore() > next_node->GetfScore()) {
          itr->second->SetfScore(next_node->GetfScore());
          NodePtr temp_itr = open_pq_.top();
          open_pq_.pop();
          open_pq_.emplace(temp_itr);
        }
      }
    }
  }
}

void AStar::GetPath() {
  path_.clear();
  if (terminaterPtr_ == nullptr) return;
  while (terminaterPtr_ != nullptr) {
    path_.push_back(terminaterPtr_);
    terminaterPtr_ = terminaterPtr_->cameFrom;
  }
  std::reverse(path_.begin(), path_.end());
}

void AStar::PrintPath() {
  if (path_.empty()) {
    std::cout << "A Star Search Failed!" << std::endl;
  } else {
    for (int i = 0; i < path_.size() - 1; ++i) {
      std::cout << "(" << path_[i]->x << ", " << path_[i]->y << ")" << " --> " << std::endl; 
    }
    std::cout << "(" << path_.back()->x << ", " << path_.back()->y << ")" << std::endl;
  }
}

double AStar::ComputeHScore(const NodePtr& node,
                            const NodePtr& goal) {
  double hScore = 0.0;
  // Manhattan
  hScore = std::fabs(node->x - goal->x) +
           std::fabs(node->y - goal->y);
  // // Euclidean
  // hScore = std::hypot(node->x - goal->x, node->y - goal->y);
  // // Dijkstra
  // hScore = 0.0;
  return hScore;
}

double AStar::ComputeGScore(const NodePtr& from,
                            const NodePtr& to) {
  return hypot(from->x - to->x, from->y - to->y);
}

std::vector<NodePtr> AStar::GetNodeNeighbor(const NodePtr& node) {
  std::vector<NodePtr> next_nodes;
  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      NodePtr new_node = new Node(node->GetX() + i, node->GetY() + j, node);
      if (*new_node == *node || IsOccupied(new_node)) {
        continue;
      }
      next_nodes.push_back(new_node);
    }
  }
  return next_nodes;
}

bool AStar::IsOccupied(const NodePtr& node) {
  if (node->GetX() < 0 || node->GetX() > map_max_x_ ||
      node->GetY() < 0 || node->GetY() > map_max_y_) {
    return false;
  }
  return gridmap_[node->GetX()][node->GetY()] > 1e-5;
}

} // namespace astar