/**
 * @file test_dijikestra_planner.cu
 */
int main(){
  float sx = 10.0;
  float sy = 10.0;
  float gx = 50.0;
  float gy = 50.0;

  float grid_size = 1.0;
  float robot_size = 1.0;

  vector<float> ox;
  vector<float> oy;

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

  dijkstra_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size);
  return 0;
}
