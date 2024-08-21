#include "../include/a_star_planner.cuh"

__device__ void computeFinalPath(Node * goal, float step, float* rx, float* ry, int* path_size)
{
  int32_t idx = 0;
  Node* node = goal;
  while (node->p_node != nullptr){
    rx[idx] = node->x * step;
    ry[idx] = node->y * step;
    node = node->p_node;
    idx++;
  };
  *path_size = idx;
};

__device__ void computeObstacleMap(
    const int32_t* ox, const int32_t* oy, int32_t num_obstacles,
    const int32_t min_ox, const int32_t max_ox,
    const int32_t min_oy, const int32_t max_oy,
    float step, float vr, int32_t* obmap)
{

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

__device__ void  verifyNode(Node* node, 
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

__device__ void computeHeuristic(Node* n1, Node* n2, float w, double* hfun) {
    *hfun = w * sqrt((n1->x - n2->x) * (n1->x - n2->x) + (n1->y - n2->y) * (n1->y - n2->y));
};

__device__ std::vector<Node> getMotionModel(){
  return {Node(1,   0,  1),
          Node(0,   1,  1),
          Node(-1,   0,  1),
          Node(0,   -1,  1),
          Node(-1,   -1,  std::sqrt(2)),
          Node(-1,   1,  std::sqrt(2)),
          Node(1,   -1,  std::sqrt(2)),
          Node(1,    1,  std::sqrt(2))};
}

__global__ void a_star_planner(double sx, double sy,
                              double gx,  double gy,
                              float* ox_, float* oy_,
                              float* step, float* rr){

  Node* nstart = new Node((int32_t)round(sx / *step), (int32_t)round(sy / *step), 0.0);
  Node* ngoal = new Node((int32_t)round(gx / *step), (int32_t)round(gy / *step), 0.0);


  vector<int32_t> ox;
  vector<int32_t> oy;

  int32_t min_ox = std::numeric_limits<int32_t>::max();
  int32_t max_ox = std::numeric_limits<int32_t>::min();
  int32_t min_oy = std::numeric_limits<int32_t>::max();
  int32_t max_oy = std::numeric_limits<int32_t>::min();


  for(float iox:ox_){
      int32_t map_x = (int32_t)std::round(iox*1.0/step);
      ox.push_back(map_x);
      min_ox = std::min(map_x, min_ox);
      max_ox = std::max(map_x, max_ox);
  }

  for(float ioy:oy_){
      int32_t map_y = (int32_t)std::round(ioy*1.0/step);
      oy.push_back(map_y);
      min_oy = std::min(map_y, min_oy);
      max_oy = std::max(map_y, max_oy);
  }

  int32_t xwidth = max_ox-min_ox;
  int32_t ywidth = max_oy-min_oy;










  //visualization
  cv::namedWindow("astar", cv::WINDOW_NORMAL);
  int32_t count = 0;
  int32_t img_reso = 5;
  cv::Mat bg(img_reso*xwidth,
             img_reso*ywidth,
             CV_8UC3,
             cv::Scalar(255,255,255));

    cv::rectangle(bg,
                  cv::Point(nstart->x*img_reso+1, nstart->y*img_reso+1),
                  cv::Point((nstart->x+1)*img_reso, (nstart->y+1)*img_reso),
                  cv::Scalar(255, 0, 0), -1);
    cv::rectangle(bg,
                  cv::Point(ngoal->x*img_reso+1, ngoal->y*img_reso+1),
                  cv::Point((ngoal->x+1)*img_reso, (ngoal->y+1)*img_reso),
                  cv::Scalar(0, 0, 255), -1);

  std::vector<std::vector<int32_t> > visit_map(xwidth, vector<int32_t>(ywidth, 0));
  
  std::vector<std::vector<float> > path_cost(xwidth, vector<float>(ywidth, std::numeric_limits<float>::max()));

  path_cost[nstart->x][nstart->y] = 0;

  std::vector<std::vector<int32_t> > obmap = computeObstacleMap(
                                                  ox, oy,
                                                  min_ox, max_ox,
                                                  min_oy, max_oy,
                                                  step, rr,
                                                  bg, img_reso);

  // NOTE: d_ary_heap should be a better choice here
  auto cmp = [](const Node* left, const Node* right){return left->sum_cost > right->sum_cost;};
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> pq(cmp);

  pq.push(nstart);
  std::vector<Node> motion = getMotionModel();

  while (true){
    Node * node = pq.top();

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
      Node * new_node = new Node(
        node->x + motion[i].x,
        node->y + motion[i].y,
        path_cost[node->x][node->y] + motion[i].sum_cost + calc_heristic(ngoal, node),
        node);

      if (!verify_node(new_node, obmap, min_ox, max_ox, min_oy, max_oy)){
        delete new_node;
        continue;
      }

      if (visit_map[new_node->x-min_ox][new_node->y-min_oy]){
        delete new_node;
        continue;
      }

      cv::rectangle(bg,
                    cv::Point(new_node->x*img_reso+1, new_node->y*img_reso+1),
                    cv::Point((new_node->x+1)*img_reso, (new_node->y+1)*img_reso),
                    cv::Scalar(0, 255, 0));

      // std::string int_count = std::to_string(count);
      // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
      count++;
      cv::imshow("astar", bg);
      cv::waitKey(5);

      if (path_cost[node->x][node->y]+motion[i].sum_cost < path_cost[new_node->x][new_node->y]){
        path_cost[new_node->x][new_node->y]=path_cost[node->x][node->y]+motion[i].sum_cost; 
        pq.push(new_node);
      }
    }
  }

  computeFinalPath(ngoal, step, bg, img_reso);
  delete ngoal;
  delete nstart;

  // std::string int_count = std::to_string(count);
  // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
  cv::imshow("astar", bg);
  cv::waitKey(5);
};


int main(){
  float sx = 10.0;
  float sy = 10.0;
  float gx = 50.0;
  float gy = 50.0;

  float grid_size = 1.0;
  float robot_size = 1.0;

  std::vector<float> ox;
  std::vector<float> oy;

  // add edges
  for(float i=0; i<60; i++){
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for(float i=0; i<60; i++){
    ox.push_back(60.0);
    oy.push_back(i);
  }
  for(float i=0; i<61; i++){
    ox.push_back(i);
    oy.push_back(60.0);
  }
  for(float i=0; i<61; i++){
    ox.push_back(0.0);
    oy.push_back(i);
  }
  for(float i=0; i<40; i++){
    ox.push_back(20.0);
    oy.push_back(i);
  }
  for(float i=0; i<40; i++){
    ox.push_back(40.0);
    oy.push_back(60.0 - i);
  }

  a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size);
  return 0;
}
