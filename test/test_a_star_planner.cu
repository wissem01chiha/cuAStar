/**
 * @file test_a_star_planner.cu 
 * test routines for a_star_planner
 * // "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
// cl /EHsc main.cpp 
 */



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
