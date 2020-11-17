#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    start_node->visited = true;

    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto node : current_node->neighbors) {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        
        open_list.push_back(node);
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::CompareNodes);

    auto next_node = open_list.back();
    open_list.pop_back();

    return next_node;
}


bool RoutePlanner::CompareNodes(const RouteModel::Node *a, const RouteModel::Node *b) {
    float cost_a = a->g_value + a->h_value;
    float cost_b = b->g_value + b->h_value;

    return cost_a > cost_b;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node != start_node) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));

        current_node = current_node->parent;
    }
    path_found.push_back(*current_node); // current_node == start_node at completion of while-loop

    // Reverse the path found above, since we started from the end and reached the beginning but we want
    // the path to start at the beginning and reach the end
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    AddNeighbors(current_node);

    while(!open_list.empty()) {
        current_node = NextNode();

        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
    }
}