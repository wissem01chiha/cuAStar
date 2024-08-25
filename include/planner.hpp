/**
 * @file planner.hpp
 * provide an unifided inetrafce for lib traj planners alorithms.
 * virtual class 
 */

enum PLANNER {
  PSO,
  BSPLINE,
  PF,
  RRT,
  ASTAR,
  RRTSTAR,
  DIJKSTRA
};

namespace internal{

class Planner
{
private:
    /* data */
public:
    Planner(/* args */);
    ~Planner();
};
}; //namespace internal