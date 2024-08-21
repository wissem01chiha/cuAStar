




class Node
{
  public:
    int32_t x;
    int32_t y;
    double  sum_cost;
    Node*   p_node;

  __device__ Node(int32_t x_, int32_t y_, 
                  float sum_cost_=0, 
                  Node* p_node_=nullptr):x(x_),y(y_),
                  sum_cost(sum_cost_),p_node(p_node_){};
};

