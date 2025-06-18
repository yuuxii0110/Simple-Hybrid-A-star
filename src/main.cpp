#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include "constant.hpp"
#include "utils.hpp"
#include "obstacle.hpp"
#include "node.hpp"
#include "bicycle_model.hpp"
#include "maze.hpp"
#include "a_star_optimizer.hpp"
#include "visualizer.hpp"

int main(int argc, char* argv[]) {
    std::cout << "=== Improved Maze Navigation with Bicycle Model Robot ===" << std::endl;

    bool no_heuristic = false;
    unsigned int seed = 10;

    if (argc == 3) {
        no_heuristic = static_cast<bool>(std::atoi(argv[1]));
        seed = std::atoi(argv[2]);
    } else {
        std::cerr << "Usage: " << argv[0] << " <seed_flag (0 or 1)>" << std::endl;   
    }

    // Initialize maze with circular obstacles and start/goal
    std::cout << "\n1. Generating maze with circular obstacles..." << std::endl;
    Maze maze(seed, no_heuristic);

    // Get initial state
    BicycleModel initial_state = maze.GetStartState();
    std::cout << "Initial state: (" << initial_state.GetX() << ", " << initial_state.GetY()
              << ", " << initial_state.GetTheta() * 180.0f / M_PI << "°)" << std::endl;

    // Perform A* search
    std::string algo_name = no_heuristic ? "Diskstra" : "A*";
    std::cout << "\n2. Starting " << algo_name << " search with arc length cost..." << std::endl;
    AStarOptimizer optimizer;

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<BicycleModel> path = optimizer.Search(initial_state, maze);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Search completed in " << duration.count() << " ms" << std::endl;

    // Visualize results
    std::cout << "\n3. Creating visualization..." << std::endl;
    Visualizer::SaveAnimation(maze, path);
    Visualizer::SaveHeuristicMap(maze);

    return 0;
}