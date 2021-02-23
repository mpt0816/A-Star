#ifndef ASTAR_SRC_NODE_H_
#define ASTAR_SRC_NODE_H_

#include <cfloat>
#include <string>

namespace astar{

struct Node {
  int x;
  int y;
  double gScore;
  double hScore;
  double fScore;
  Node* cameFrom;
  std::string index;

  Node(const int _x, const int _y) 
      : x(_x)
      , y(_y)
      , gScore(0.0)
      , hScore(0.0)
      , fScore(0.0)
      , cameFrom(NULL) {
        index = ComputeIndex(x, y);
      }
  Node(const int _x, const int _y, Node* ptr)
      : x(_x)
      , y(_y)
      , gScore(0.0)
      , hScore(0.0)
      , fScore(0.0)
      , cameFrom(ptr) {
    index = ComputeIndex(x, y);
  }

  void SetgScore(const double g) {
    gScore = g;
    fScore = gScore + hScore;
  }

  void SethScore(const double h) {
    hScore = h;
    fScore = gScore + hScore;
  }

  void SetfScore(const double f) {
    fScore = f;
  }

  void SetCameFrom(Node* ptr) {
    cameFrom = ptr;
  }

  int GetX() const {
    return x;
  }

  int GetY() const {
    return y;
  }

  double GetgScore() const {
    return gScore;
  }

  double GethScore() const {
    return hScore;
  }

  double GetfScore() const {
    return fScore;
  }

  bool operator==(const Node& node) const {
    return (x == node.x && y == node.y);
  }

  std::string GetIndex() const {
    return index;
  }

  std::string ComputeIndex(const int x, const int y) {
    return std::to_string(x) + "_" + std::to_string(y);
  }

};

typedef Node* NodePtr;

} // namespace astar

#endif // ASTAR_SRC_NODE_H_


