#pragma once

#include "maze.hpp"

class AStarOptimizer {
private:
    std::vector<float> GetDeltaSteering(float current_steering) const {
        std::vector<float> inputs;
        inputs.reserve(3);

        if (current_steering < MAX_STEERING_ANGLE - 1e-3) {
            inputs.push_back(MAX_STEERING_RATE*DT);
        }

        inputs.push_back(0.F);

        if (current_steering > -MAX_STEERING_ANGLE + 1e-3) {
            inputs.push_back(-MAX_STEERING_RATE*DT);
        }

        return inputs;
    }
    
    std::vector<BicycleModel> ReconstructPath(const Maze& maze, size_t goal_key) const {
        std::vector<BicycleModel> path;
        size_t current_key = goal_key;
        
        while (current_key != SIZE_MAX) {
            const Node& node = maze.GetBestNode(current_key);
            path.push_back(node.state_);
            current_key = node.parent_key_;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
public:
    std::vector<BicycleModel> Search(const BicycleModel& initial_state, Maze& maze) const {
        std::priority_queue<std::pair<float, size_t>, 
                           std::vector<std::pair<float, size_t>>, 
                           std::greater<>> open_set;
        
        size_t start_key = initial_state.GetStateKey();
        Node start_node(initial_state, 0.0F, maze.GetHeuristic(initial_state), SIZE_MAX);
        
        maze.UpdateBestNode(start_key, start_node);
        open_set.push({start_node.f_score_, start_key});
        
        int iterations = 0;
        const int MAX_ITERATIONS = 500000;
        std::unordered_set<size_t> visited;
        
        while (!open_set.empty() && iterations < MAX_ITERATIONS) {
            iterations++;
            
            const size_t current_key = open_set.top().second;
            open_set.pop();
            
            if (!maze.HasBestNode(current_key)) {
                // something is wrong, it exists in the queue but not in the best_nodes_ map
                throw std::runtime_error("Node not found in best_nodes map but is in the open_set");
            }

            if (visited.count(current_key)) {
                continue;  // Skip already visited nodes
            }
            
            visited.insert(current_key);
            const Node& current_node = maze.GetBestNode(current_key);
            
            // Check if goal reached
            if (maze.IsGoalReached(current_node.state_)) {
                std::cout << "Path found after " << iterations << " iterations!" << std::endl;
                std::cout << "Path cost: " << current_node.cost_ << std::endl;
                return ReconstructPath(maze, current_key);
            }
            
            // Expand neighbors
            for (const auto& steering_input : GetDeltaSteering(current_node.state_.GetSteeringAngle())) {
                const auto next_state = current_node.state_.Move(steering_input);

                if (!next_state.IsValid() || !maze.IsValidState(next_state)) {
                    continue;
                }
                const auto arc_length = next_state.ComputeArcLengthFromParent(current_node.state_);

                // Use arc length as the g-cost increment
                float tentative_cost = current_node.cost_ + arc_length;
                size_t next_key = next_state.GetStateKey();
                
                if (!maze.HasBestNode(next_key) || 
                    tentative_cost < maze.GetBestNode(next_key).cost_) {
                    
                    float heuristic = maze.GetHeuristic(next_state);
                    float f_score = tentative_cost + heuristic;
                    
                    Node next_node(next_state, tentative_cost, f_score, current_key);
                    maze.UpdateBestNode(next_key, next_node);
                    open_set.push({f_score, next_key});
                }
            }
        }
        
        std::cout << "Search completed after " << iterations << " iterations. No path found." << std::endl;
        return std::vector<BicycleModel>();
    }
};
