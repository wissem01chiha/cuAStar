/** 
 * @file a_star_planner.cu 
 */
#define ENABLE_CUDA_ARCH 1
#include "../include/a_star_planner.hpp"

__device__  AstarPlanner::AstarPlanner(){};

__global__ void AstarPlanner::computeFinalPath(internal::Node2d* goal,double step,double* rx,
                                            double* ry,int* path_size){
  int32_t idx = 0;
  internal::Node2d* node = goal;
  while (node->p_node != nullptr){
    rx[idx] = node->x * step;
    ry[idx] = node->y * step;
    node = node->p_node;
    idx++;
  };
  *path_size = idx;
};

__global__ void AstarPlanner::computeObstacleMap(const int32_t* ox, const int32_t* oy, 
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

__device__ void  AstarPlanner::verifyNode(internal::Node2d* node, 
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

__device__ void AstarPlanner::computeHeuristic(internal::Node2d* n1,internal::Node2d* n2,
                                            double w,double* hfun){
    *hfun = w * sqrt((n1->x-n2->x)*(n1->x-n2->x)+(n1->y-n2->y)*(n1->y-n2->y));
};

__device__ void AstarPlanner::computeMotionModel(internal::Node2d* motion_model){
    
    motion_model[0] = internal::Node2d(1, 0, 1);
    motion_model[1] = internal::Node2d(0, 1, 1);
    motion_model[2] = internal::Node2d(-1, 0, 1);
    motion_model[3] = internal::Node2d(0, -1, 1);
    motion_model[4] = internal::Node2d(-1, -1, sqrt(2.0));
    motion_model[5] = internal::Node2d(-1, 1, sqrt(2.0));
    motion_model[6] = internal::Node2d(1, -1, sqrt(2.0));
    motion_model[7] = internal::Node2d(1, 1, sqrt(2.0));
};

__global__ void AstarPlanner::computeTrajectory(double sx, double sy, double gx, double gy,
                                  double* ox_, double* oy_, int n,
                                  double step, double rr,
                                  int32_t* visit_map, double* path_cost,
                                  internal::Node2d* motion_model, internal::Node2d* traj_nodes) {

    // Initialize start and goal nodes
    internal::Node2d nstart = { (int32_t)round(sx / step), (int32_t)round(sy / step), 0.0, nullptr };
    internal::Node2d ngoal = { (int32_t)round(gx / step), (int32_t)round(gy / step), 0.0, nullptr };

    // Compute min and max values for obstacle coordinates
    int32_t min_ox = INT_MAX, max_ox = INT_MIN;
    int32_t min_oy = INT_MAX, max_oy = INT_MIN;

    computeUpdateMinMax(ox_, step, &min_ox, &max_ox, n);
    computeUpdateMinMax(oy_, step, &min_oy, &max_oy, n);

    int32_t xwidth = max_ox - min_ox + 1;
    int32_t ywidth = max_oy - min_oy + 1;

    // Initialize visit map and path cost arrays
    int32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < xwidth * ywidth) {
        visit_map[tid] = 0;
        path_cost[tid] = DBL_MAX;
    }
    __syncthreads();

    // Set the start node's cost
    if (nstart.x >= min_ox && nstart.y >= min_oy) {
        path_cost[(nstart.x - min_ox) * ywidth + (nstart.y - min_oy)] = 0.0;
    }

    // Initialize a simplified priority queue (manual implementation)
    int queue_head = 0;
    traj_nodes[queue_head] = nstart;

    while (queue_head >= 0) {
        internal::Node2d node = traj_nodes[queue_head--];

        int node_idx = (node.x - min_ox) * ywidth + (node.y - min_oy);
        if (visit_map[node_idx] == 1) continue;

        visit_map[node_idx] = 1;

        if (node.x == ngoal.x && node.y == ngoal.y) {
            ngoal.sum_cost = node.sum_cost;
            ngoal.p_node = node.p_node;
            break;
        }

        // Evaluate neighbors
        for (int i = 0; i < 8; i++) { // 8 possible moves
            internal::Node2d new_node = { node.x + motion_model[i].x, node.y + motion_model[i].y,
                              node.sum_cost + motion_model[i].sum_cost, &traj_nodes[queue_head + 1] };

            if (new_node.x < min_ox || new_node.y < min_oy || new_node.x >= max_ox || new_node.y >= max_oy) continue;

            int new_idx = (new_node.x - min_ox) * ywidth + (new_node.y - min_oy);
            if (visit_map[new_idx] == 1) continue;

            if (path_cost[new_idx] > new_node.sum_cost) {
                path_cost[new_idx] = new_node.sum_cost;
                traj_nodes[++queue_head] = new_node;  // Push new node to the queue
            }
        }
    }

    // Post-processing to extract the trajectory (simplified)
    // ... (fill traj_nodes to retrieve the final path)

};