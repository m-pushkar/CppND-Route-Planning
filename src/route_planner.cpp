#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.

  // Find closet nodes for given coordinates
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
  // Find distance between given 2 nodes
  return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Populate current node's neighbor vector
  current_node->FindNeighbors();
  
  // Go through each neighbor, calculate attributes and add it to the open list
  for (auto neighbor : current_node->neighbors){
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
  
  //Calculate and compare f values
  auto f1 = a->g_value + a->h_value;
  auto f2 = b->g_value + b->h_value;
  return f1 > f2;
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  
  // Sort open nodes list by f values
  std::sort (open_list.begin(), open_list.end(), Compare);
  
  // Extract node with lowest f value and remove it from list
  auto next_node = open_list.back();
  open_list.pop_back();
  next_node->visited = true;
  return next_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Repeat till we reach the starting point
  while (current_node) {
    path_found.push_back(*current_node);
    auto next_parent = current_node->parent;
    if (next_parent) {
      distance += next_parent->distance(*current_node);
    }
    current_node = next_parent;
  }
  
  // Reverse the path_found list
  std::reverse(path_found.begin(), path_found.end());
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  
  open_list = {};
  open_list.push_back(start_node);
  
  while (open_list.size() > 0) {
    if (current_node == end_node) {
      break;
    }
    this->AddNeighbors(current_node);
  }
  m_Model.path = ConstructFinalPath(end_node);
}