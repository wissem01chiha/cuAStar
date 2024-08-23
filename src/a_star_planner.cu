/** 
 * @file a_star_planner.cu 
 */
#define ENABLE_CUDA_ARCH 1
#include "../include/a_star_planner.hpp"

__device__ void AstarPlanner::computeFinalPath(internal::Node* goal,double step,double* rx,
                                            double* ry,int* path_size){
  int32_t idx = 0;
  internal::Node* node = goal;
  while (node->p_node != nullptr){
    rx[idx] = node->x * step;
    ry[idx] = node->y * step;
    node = node->p_node;
    idx++;
  };
  *path_size = idx;
};

__device__ void AstarPlanner::computeObstacleMap(const int32_t* ox, const int32_t* oy, 
                                              int32_t num_obstacles, const int32_t min_ox, 
                                              const int32_t max_ox, const int32_t min_oy, 
                                              const int32_t max_oy, double step, double vr, 
                                              int32_t* obmap){
  int32_t xwidth = max_ox-min_ox;
  int32_t ywidth = max_oy-min_oy;

  int32_t i = blockIdx.x * blockDim.x + threadIdx.x;  
  int32_t j = blockIdx.y * blockDim.y + threadIdx.y;

  if(i < xwidth &&  j < ywidth)
  {
    int32_t x = i + min_ox;
    int32_t y = j + min_oy;

    for (int k = 0; k < num_obstacles; k++){
            double d = sqrtf(powf((ox[k] - x), 2) + powf((oy[k] - y), 2));
            if (d <= vr / step) {
                obmap[j * xwidth + i] = 1;
                break;
            }
      }
    }
  };

__device__ void  AstarPlanner::verifyNode(internal::Node* node, 
                          const int32_t* obmap,
                          int32_t min_ox, 
                          int32_t max_ox,
                          int32_t min_oy, 
                          int32_t max_oy,
                          int32_t xwidth,
                          bool* state){
  if (node->x < min_ox || node->y < min_oy || node->x >= max_ox || node->y >= max_oy){
    *state =  false;
  };
    int idx_x = node->x - min_ox;
    int idx_y = node->y - min_oy;
    int index = idx_y * xwidth + idx_x;
    if (obmap[index]) {
        *state = false;
    } else {
        *state = true;
    }
};

__device__ void AstarPlanner::computeHeuristic(internal::Node* n1,internal::Node* n2,
                                            double w,double* hfun){
    *hfun = w * sqrt((n1->x-n2->x)*(n1->x-n2->x)+(n1->y-n2->y)*(n1->y-n2->y));
};

__device__ void AstarPlanner::computeMotionModel(internal::Node* motion_model){
    
    motion_model[0] = internal::Node(1, 0, 1);
    motion_model[1] = internal::Node(0, 1, 1);
    motion_model[2] = internal::Node(-1, 0, 1);
    motion_model[3] = internal::Node(0, -1, 1);
    motion_model[4] = internal::Node(-1, -1, sqrt(2.0));
    motion_model[5] = internal::Node(-1, 1, sqrt(2.0));
    motion_model[6] = internal::Node(1, -1, sqrt(2.0));
    motion_model[7] = internal::Node(1, 1, sqrt(2.0));
};

__global__ void AstarPlanner::computeTrajectory(double sx, double sy,
                                            double gx,  double gy,
                                            double* ox_, double* oy_,
                                            double* step, double* rr, 
                                            double * traj){

  internal::Node* nstart = new internal::Node((int32_t)round(sx/ *step),
                            (int32_t)round(sy/ *step),0.0);
  internal::Node* ngoal = new internal::Node((int32_t)round(gx/ *step),
                           (int32_t)round(gy/ *step),0.0);

  int32_t ox[n];
  int32_t oy[n];
  
  int32_t *min_ox = new int32_t;
  int32_t *max_ox = new int32_t;
  int32_t *min_oy = new int32_t;
  int32_t *max_oy = new int32_t;

  *min_ox = -2147483647;
  *max_ox = 2147483648;
  *min_oy = 2147483647;
  *max_oy = -2147483648;

  computeUpdateMinMax(ox_,*step,min_ox,max_ox,n);
  computeUpdateMinMax(oy_,*step,min_oy,max_oy,n);

  int32_t xwidth = max_ox-min_ox;
  int32_t ywidth = max_oy-min_oy;

  int32_t count = 0;
  int32_t img_reso = 5;


  std::vector<std::vector<int32_t>> visit_map(xwidth, vector<int32_t>(ywidth, 0));
  
  std::vector<std::vector<double>> path_cost(xwidth, vector<double>(ywidth, std::numeric_limits<double>::max()));

  path_cost[nstart->x][nstart->y] = 0;

  computeObstacleMap(ox, oy, min_ox, max_ox, min_oy, max_oy,step, rr,bg, img_reso);

  
  auto cmp = [](const Node* left, const Node* right){return left->sum_cost > right->sum_cost;};
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);

  pq.push(nstart);
  internal::Node* motion_model;
  computeMotionModel(motion_model);

  while (true){
    internal::Node * node = pq.top();

    if (visit_map[node->x-min_ox][node->y-min_oy] == 1){
      pq.pop();
      delete node;
      continue;
    }else{
      pq.pop();
      visit_map[node->x-min_ox][node->y-min_oy] = 1;
    }

    if (node->x == ngoal->x && node->y==ngoal->y){
      ngoal->sum_cost = node->sum_cost;
      ngoal->p_node = node;
      break;
    }

    for(int32_t i=0; i<motion.size(); i++){
      internal::Node * new_node = new internal::Node(
        node->x + motion[i].x,
        node->y + motion[i].y,
        path_cost[node->x][node->y] + motion[i].sum_cost + computeHeuristic(ngoal, node),
        node);
        bool* state;
      verifyNode(node, obmap,min_ox, max_ox,min_oy,max_oy,xwidth,state);

      if (!state){
        delete new_node;
        continue;
      }

      if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
        delete new_node;
        continue;
      }
      count++;
      if (path_cost[node->x][node->y]+motion[i].sum_cost < path_cost[new_node->x][new_node->y]){
        path_cost[new_node->x][new_node->y]=path_cost[node->x][node->y]+motion[i].sum_cost; 
        pq.push(new_node);
      }
    }
  }
  computeFinalPath(ngoal, step, bg, img_reso);
  delete ngoal;
  delete nstart;
};