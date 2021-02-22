#ifndef ASTAR_SRC_SEARCH_H_
#define ASTAR_SRC_SEARCH_H_

#include "node.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <eigen3/Eigen/Core>

namespace astar {

class AStar {
 public:
  AStar();
  ~AStar() {};
  void InitGridMap(const Eigen::MatrixXd& map);
  void PathSearch(const Eigen::Vector2i& start_pt,
                  const Eigen::Vector2i& end_pt);
  void GetPath();
  void PrintPath();

 private:
  double ComputeHScore(const NodePtr& node,
                       const NodePtr& goal);
  double ComputeGScore(const NodePtr& from,
                       const NodePtr& to);
  std::vector<NodePtr> GetNodeNeighbor(const NodePtr& node);
  bool IsOccupied(const NodePtr& node);
  struct pq_cmp {
    bool operator()(const NodePtr& left, const NodePtr& right) const {
      return left->GetfScore() >= right->GetfScore();
    }
  };
 private:
  double** gridmap_;
  NodePtr terminaterPtr_;
  std::vector<NodePtr> path_;
  int map_max_x_;
  int map_max_y_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, pq_cmp> open_pq_;
  std::unordered_map<std::string, NodePtr> open_map_;
  std::unordered_map<std::string, NodePtr> close_map_;
};

} // namespace astar

#endif // ASTAR_SRC_SEARCH_H_
