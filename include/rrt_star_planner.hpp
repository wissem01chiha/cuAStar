#ifndef RRT_STAR_PLANNER_HPP
#define RRT_STAR_PLANNER_HPP

#include<iostream>
#include<limits>
#include<random>
#include<vector>
#include<cmath>

#include "common.hpp"
#include "rrt_planner.hpp"

 
// class RRTStar_Node : public internal::Node2d{
// public:
//   float cost;
// 	RRTStar_Node(float x_, float y_): internal::Node2d(x_, y_), cost(0){};
// };

class RRTStar: public RRT{
public:
	RRTStar(internal::Node2d*, internal::Node2d*, std::vector<std::vector<float> >, 
  std::vector<float>, float, float, int, int, float);

  std::vector<internal::Node2d* > planning();
private:
  std::vector<internal::Node2d* > find_near_nodes(internal::Node2d* );

  float calc_new_cost(internal::Node2d*, internal::Node2d*);

  internal::Node2d* choose_parent(internal::Node2d*, std::vector<internal::Node2d* >);

  float connect_circle_dist;
};

RRTStar::RRTStar(internal::Node2d* start_, internal::Node2d* end_,
std::vector<std::vector<float> > ob_list_, std::vector<float> rand_area_, float expand_dis_, 
float path_resolution_, int goal_sample_rate_, int max_iter_, float connect_circle_dist_): RRT(start_, end_, ob_list_, rand_area_, expand_dis_, path_resolution_, goal_sample_rate_, max_iter_), connect_circle_dist(connect_circle_dist_){};

internal::Node2d* RRTStar::choose_parent(internal::Node2d* new_node, std::vector<internal::Node2d* > neighbours){
  internal::Node2d* output;
  if (neighbours.size()==0){
    return output;
  }

  std::vector<float> costs;
  for (internal::Node2d* n_:neighbours){
    internal::Node2d* t_node = steer(n_, new_node);
    if (t_node && CollisionCheck(t_node)){
      costs.push_back(calc_new_cost(n_, new_node));
    }else{
      costs.push_back(std::numeric_limits<float>::max());
    }
  }
  // min_costs

};

std::vector<internal::Node2d* > RRTStar::planning(){
	node_list.push_back(start);
  for(int i=0; i<max_iter; i++){
		std::vector<float> rnd;
		if (goal_dis(goal_gen)>goal_sample_rate){
			float rand_x = area_dis(goal_gen);
			float rand_y = area_dis(goal_gen);
			rnd.push_back(rand_x);
			rnd.push_back(rand_y);
		}else{
			rnd.push_back(end->x);
			rnd.push_back(end->y);
		}

    internal::Node2d* rnd_node = new internal::Node2d(rnd[0], rnd[1]);
		internal::Node2d* nearest_node =  GetNearestNode(rnd);

    internal::Node2d* new_node = steer(nearest_node, rnd_node, expand_dis);
    
		if (!CollisionCheck(new_node)) continue;
      
    std::vector<internal::Node2d* > neighbour_nodes=find_near_nodes(new_node);
    // choose parent  

  return node_list;

  }
};

std::vector<internal::Node2d* > RRTStar::find_near_nodes(internal::Node2d* new_node){
  std::vector<internal::Node2d* > output;
  int nnode = node_list.size() + 1;
  float r = connect_circle_dist * std::sqrt(std::log(nnode)/nnode);
  for(internal::Node2d* n_:node_list){
    if ((n_->x-new_node->x)*(n_->x-new_node->x) + (n_->y-new_node->y)*(n_->y-new_node->y)<r*r){
      output.push_back(n_);
    }
    return output;
  }
};

float RRTStar::calc_new_cost(internal::Node2d* from_node, internal::Node2d* to_node){
  std::vector<float> dist_angle = calc_distance_and_angle(from_node, to_node);
  return from_node->cost + dist_angle[0];
};

 
#endif
