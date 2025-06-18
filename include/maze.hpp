#pragma once

#include "constant.hpp"
#include "bicycle_model.hpp"

class Maze {
private:
    std::vector<uint8_t> obstacles_;  // obstacles_[y * MAZE_SIZE + x]
    std::vector<float> heuristic_map_;  // heuristic_map_[y * MAZE_SIZE + x]
    std::vector<CircularObstacle> circular_obstacles_;
    std::unordered_map<size_t, Node> best_nodes_;
    std::mt19937 rng_;
    int start_x_, start_y_, goal_x_, goal_y_;
    bool no_heuristic_;
    
    void GenerateCircularObstacles() {
        // Initialize all as free space
        obstacles_.assign(MAZE_SIZE * MAZE_SIZE, 0);
        
        // Add border walls
        for (int i = 0; i < MAZE_SIZE; i++) {
            obstacles_[0 * MAZE_SIZE + i] = 1;  // top border
            obstacles_[(MAZE_SIZE-1) * MAZE_SIZE + i] = 1;  // bottom border
            obstacles_[i * MAZE_SIZE + 0] = 1;  // left border
            obstacles_[i * MAZE_SIZE + (MAZE_SIZE-1)] = 1;  // right border
        }
        
        // Generate random start and goal positions first
        std::uniform_int_distribution<int> pos_dist(CLEAR_RADIUS + 2, MAZE_SIZE - CLEAR_RADIUS - 3);
        start_x_ = pos_dist(rng_);
        start_y_ = pos_dist(rng_);

        do {
            goal_x_ = pos_dist(rng_);
            goal_y_ = pos_dist(rng_);
        } while (obstacles_[goal_y_ * MAZE_SIZE + goal_x_] || 
                (std::abs(goal_x_ - start_x_) < 20 && std::abs(goal_y_ - start_y_) < 20));
        
        // Generate random circular obstacles
        std::uniform_int_distribution<int> num_obstacles_dist(MIN_OBSTACLES, MAX_OBSTACLES);
        std::uniform_int_distribution<int> radius_dist(MIN_OBSTACLE_RADIUS, MAX_OBSTACLE_RADIUS);
        std::uniform_int_distribution<int> center_dist(MAX_OBSTACLE_RADIUS, MAZE_SIZE - MAX_OBSTACLE_RADIUS);
        
        int num_obstacles = num_obstacles_dist(rng_);
        
        for (int i = 0; i < num_obstacles; i++) {
            circular_obstacles_.emplace_back(center_dist(rng_), center_dist(rng_), radius_dist(rng_));
        }
        
        // Fill obstacle pixels
        for (const auto& obs : circular_obstacles_) {
            for (int y = std::max(0, obs.center_y - obs.radius); y <= std::min(MAZE_SIZE - 1, obs.center_y + obs.radius); y++) {
                for (int x = std::max(0, obs.center_x - obs.radius); x <= std::min(MAZE_SIZE - 1, obs.center_x + obs.radius); x++) {
                    if (obs.Contains(x, y)) {
                        const int dx_start = x - start_x_;
                        const int dy_start = y - start_y_;
                        const int dx_goal = x - goal_x_;
                        const int dy_goal = y - goal_y_;
                        
                        if (dx_start * dx_start + dy_start * dy_start <= CLEAR_RADIUS * CLEAR_RADIUS) {
                            continue;
                        }

                        if (dx_goal * dx_goal + dy_goal * dy_goal <= CLEAR_RADIUS * CLEAR_RADIUS) {
                            continue;
                        }

                        obstacles_[y * MAZE_SIZE + x] = 1;
                    }
                }
            }
        }
    }
    
    // this method run diskstra from goal to all the x,y state without considering motion constraints
    void ComputeHeuristicMap() {
        if (no_heuristic_) {
            heuristic_map_.assign(MAZE_SIZE * MAZE_SIZE, 0.F);
            return;
        }

        heuristic_map_.assign(MAZE_SIZE * MAZE_SIZE, std::numeric_limits<float>::max());
        
        // Dijkstra's algorithm from goal
        std::priority_queue<std::pair<float, int>, 
                           std::vector<std::pair<float, int>>, 
                           std::greater<>> pq;
        
        int goal_idx = goal_y_ * MAZE_SIZE + goal_x_;
        heuristic_map_[goal_idx] = 0.0f;
        pq.push({0.0f, goal_idx});
        
        const std::array<int, 8> dx{-1, 1, 0, 0, -1, -1, 1, 1};
        const std::array<int, 8> dy{0, 0, -1, 1, -1, 1, -1, 1};
        const std::array<float, 8> costs{1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};
        
        while (!pq.empty()) {
            float dist = pq.top().first;
            int idx = pq.top().second;
            pq.pop();
            
            int x = idx % MAZE_SIZE;
            int y = idx / MAZE_SIZE;
            
            if (dist > heuristic_map_[idx]) continue;
            
            for (int i = 0; i < 8; i++) {
                int nx = x + dx[i];
                int ny = y + dy[i];
                
                if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
                    int nidx = ny * MAZE_SIZE + nx;
                    if (!obstacles_[nidx]) {
                        float new_dist = dist + costs[i];
                        if (new_dist < heuristic_map_[nidx]) {
                            heuristic_map_[nidx] = new_dist;
                            pq.push({new_dist, nidx});
                        }
                    }
                }
            }
        }
    }
    
public:
    Maze(unsigned int seed = std::random_device{}(), bool no_heuristic = false) : rng_(seed), no_heuristic_(no_heuristic) {
        GenerateCircularObstacles();
        ComputeHeuristicMap();
        
        std::cout << "Maze generated with " << circular_obstacles_.size() << " circular obstacles" << std::endl;
        std::cout << "Start: (" << start_x_ << ", " << start_y_ << ")" << std::endl;
        std::cout << "Goal: (" << goal_x_ << ", " << goal_y_ << ")" << std::endl;
        std::cout << "DT: " << DT << " seconds" << std::endl;
        std::cout << "Theta bins: " << THETA_BINS << " (18° per bin)" << std::endl;
    }
    
    bool IsObstacle(int x, int y) const {
        if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return true;
        return obstacles_[y * MAZE_SIZE + x] != 0;
    }
    
    bool IsValidState(const BicycleModel& model) const {
        return !IsObstacle(model.GetGridX(), model.GetGridY());
    }
    
    float GetHeuristic(const BicycleModel& model) const {
        // return dist to goal
        if (no_heuristic_) {
            return 0.0f;
        }

        int x = model.GetGridX();
        int y = model.GetGridY();
        if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) {
            return std::numeric_limits<float>::infinity();
        }
        return heuristic_map_[y * MAZE_SIZE + x];
    }
    
    bool IsGoalReached(const BicycleModel& model) const {
        return model.GetGridX() == goal_x_ && model.GetGridY() == goal_y_;
    }
    
    void UpdateBestNode(size_t key, const Node& node) {
        auto it = best_nodes_.find(key);
        if (it == best_nodes_.end() || node.cost_ < it->second.cost_) {
            best_nodes_[key] = node;
        }
    }
    
    bool HasBestNode(size_t key) const {
        return best_nodes_.find(key) != best_nodes_.end();
    }
    
    const Node& GetBestNode(size_t key) const {
        return best_nodes_.at(key);
    }
    
    BicycleModel GetStartState() const {
        return BicycleModel(start_x_ + 0.5f, start_y_ + 0.5f, 0.0f, 0.0f);
    }
    
    int CountObstacles() const {
        int count = 0;
        for (uint8_t obstacle : obstacles_) {
            if (obstacle) count++;
        }
        return count;
    }
    
    // Getters
    int GetStartX() const { return start_x_; }
    int GetStartY() const { return start_y_; }
    int GetGoalX() const { return goal_x_; }
    int GetGoalY() const { return goal_y_; }
    const std::vector<uint8_t>& GetObstacles() const { return obstacles_; }
    const std::vector<CircularObstacle>& GetCircularObstacles() const { return circular_obstacles_; }
    const std::unordered_map<size_t, Node>& GetBestNodes() const { return best_nodes_; }
    const std::vector<float>& GetHeuristicMap() const { return heuristic_map_; }
};