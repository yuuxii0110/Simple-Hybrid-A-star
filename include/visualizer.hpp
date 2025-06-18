#pragma once

#include <opencv2/opencv.hpp>
class Visualizer {
public:
    static void SaveHeuristicMap(const Maze& maze, const std::string& filename = "heuristic_map.png") {
        const auto& heuristic = maze.GetHeuristicMap();
        cv::Mat map_img(MAZE_SIZE, MAZE_SIZE, CV_8UC1);

        for (int y = 0; y < MAZE_SIZE; ++y) {
            for (int x = 0; x < MAZE_SIZE; ++x) {
                const float h = std::min(std::max(0.F, static_cast<float>(heuristic[y * MAZE_SIZE + x])), 255.F);
                const uint8_t dh = static_cast<uint8_t>(h);
                // Invert grayscale: 0 = white (low cost), 100 = black (high cost)
                uint8_t color = static_cast<uint8_t>(255 - std::min(dh, static_cast<uint8_t>(100)) * 255 / 100);
                map_img.at<uint8_t>(y, x) = color;
            }
        }

        cv::imwrite(filename, map_img);
        std::cout << "Heuristic map saved as " << filename << std::endl;
    }
    static void SaveAnimation(const Maze& maze, const std::vector<BicycleModel>& path) {
        const int scale = 1;
        cv::Mat base_frame(MAZE_SIZE * scale, MAZE_SIZE * scale, CV_8UC3, cv::Scalar(0, 0, 0));

        // Draw rectangular maze obstacles
        const auto& obstacles = maze.GetObstacles();
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                cv::Scalar color = obstacles[y * MAZE_SIZE + x] ? 
                                   cv::Scalar(50, 50, 50) : cv::Scalar(255, 255, 255);
                cv::rectangle(base_frame, 
                              cv::Point(x * scale, y * scale), 
                              cv::Point((x + 1) * scale, (y + 1) * scale), 
                              color, -1);
            }
        }

        // Draw start and goal positions
        cv::Point start_pt(maze.GetStartX() * scale + scale/2, maze.GetStartY() * scale + scale/2);
        cv::Point goal_pt(maze.GetGoalX() * scale + scale/2, maze.GetGoalY() * scale + scale/2);

        cv::circle(base_frame, start_pt, 5*scale/2, cv::Scalar(0, 255, 0), -1);         // green dot
        cv::circle(base_frame, goal_pt, 5*scale/2, cv::Scalar(0, 0, 255), -1);          // red dot

        // If no path found
        if (path.empty()) {
            const auto& resized = ResizedFrame(base_frame);
            cv::imwrite("maze_no_path.png", resized);
            cv::imshow("Maze - No Path Found", resized);
            cv::waitKey(0);
            cv::destroyAllWindows();
            return;
        }

        // Create simulation frames
        for (size_t i = 0; i < path.size(); i++) {
            cv::Mat current_frame = base_frame.clone();

            // Draw path trajectory
            for (size_t j = 1; j < i && j < path.size(); j++) {
                cv::Point p1(static_cast<int>(path[j-1].GetX() * scale + scale/2), 
                             static_cast<int>(path[j-1].GetY() * scale + scale/2));
                cv::Point p2(static_cast<int>(path[j].GetX() * scale + scale/2), 
                             static_cast<int>(path[j].GetY() * scale + scale/2));
                cv::line(current_frame, p1, p2, cv::Scalar(0, 255, 255), 2);
            }

            // Draw robot
            cv::Point robot_pos(static_cast<int>(path[i].GetX() * scale + scale/2), 
                                static_cast<int>(path[i].GetY() * scale + scale/2));
            cv::circle(current_frame, robot_pos, scale/3, cv::Scalar(255, 0, 255), -1);
            // Draw heading
            float half_arrow_len = WHEELBASE/1.5*scale;
            cv::Point arrow_start(
                robot_pos.x - half_arrow_len * std::cos(path[i].GetTheta()),
                robot_pos.y - half_arrow_len * std::sin(path[i].GetTheta()));
            cv::Point arrow_end(
                robot_pos.x + half_arrow_len * std::cos(path[i].GetTheta()),
                robot_pos.y + half_arrow_len * std::sin(path[i].GetTheta()));
            cv::arrowedLine(current_frame, arrow_start, arrow_end, cv::Scalar(255, 255, 0), 2);

            // Save image
            std::string filename = "simulation_" + std::to_string(i) + ".png";
            cv::imwrite(filename, ResizedFrame(current_frame));
        }

        // Display final result
        cv::Mat final_frame = base_frame.clone();
        for (size_t j = 1; j < path.size(); j++) {
            cv::Point p1(static_cast<int>(path[j-1].GetX() * scale + scale/2), 
                         static_cast<int>(path[j-1].GetY() * scale + scale/2));
            cv::Point p2(static_cast<int>(path[j].GetX() * scale + scale/2), 
                         static_cast<int>(path[j].GetY() * scale + scale/2));
            cv::line(final_frame, p1, p2, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow("Final Path", ResizedFrame(final_frame));
        cv::waitKey(0);
        cv::destroyAllWindows();

        std::cout << "Saved " << path.size() << " simulation frames." << std::endl;
    }

private:
    static cv::Mat ResizedFrame(const cv::Mat& base) {
        static const cv::Size output_size(500, 500);
        cv::Mat resized_frame;
        cv::resize(base, resized_frame, output_size, 0, 0, cv::INTER_CUBIC);
        
        return resized_frame;
    }   
};