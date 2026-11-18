#pragma once
#include <memory>
namespace ego_planner {
class PlanningVisualization {
public:
  using Ptr = std::shared_ptr<PlanningVisualization>;
  template<class... T> void displayMultiInitPathList(T&&...) {}
  template<class... T> void displayInitPathList(T&&...) {}
  template<class... T> void displayOptimalList(T&&...) {}
};
}
