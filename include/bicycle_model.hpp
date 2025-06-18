#pragma once

#include "constant.hpp"

class BicycleModel {
private:
    // Continuous state
    float x_, y_, theta_, steering_angle_;
    
    // Discretized state
    int grid_x_, grid_y_, theta_bin_, steering_bin_;
    
    void UpdateDiscretizedState() {
        grid_x_ = static_cast<int>(std::floor(x_));
        grid_y_ = static_cast<int>(std::floor(y_));
        
        // Discretize theta to bins (20 bins = 18 degrees each)
        theta_bin_ = static_cast<int>(theta_ * THETA_BINS / (2 * M_PI)) % THETA_BINS;
        
        // Discretize steering angle to bins
        float normalized_steering = (steering_angle_ + MAX_STEERING_ANGLE) / (2 * MAX_STEERING_ANGLE);
        steering_bin_ = std::max(0, std::min(STEERING_BINS - 1, 
                               static_cast<int>(normalized_steering * STEERING_BINS)));
    }
    
public:
    BicycleModel(float x = 0.0f, float y = 0.0f, float theta = 0.0f, float steering = 0.0f) 
        : x_(x), y_(y), theta_(AngMod2PI(theta)), steering_angle_(AngModPI(steering)) {
        UpdateDiscretizedState();
    }
    
    BicycleModel Move(float delta_steering) const {
        // Limit steering rate
        const float new_steering = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, steering_angle_ + delta_steering));
        
        // Bicycle model dynamics
        const float beta = std::atan(0.5F * std::tan(new_steering)); // slip angle at CG
        const float new_x = x_ + ROBOT_SPEED * std::cos(theta_ + beta) * DT;
        const float new_y = y_ + ROBOT_SPEED * std::sin(theta_ + beta) * DT;
        const float new_theta = theta_ + (ROBOT_SPEED / WHEELBASE) * std::tan(new_steering) * std::cos(beta) * DT;
                
        return BicycleModel(new_x, new_y, new_theta, new_steering);
    }
    
    bool IsValid() const {
        return grid_x_ >= 0 && grid_x_ < MAZE_SIZE && grid_y_ >= 0 && grid_y_ < MAZE_SIZE;
    }
    
    size_t GetStateKey() const {
        return static_cast<size_t>(grid_x_) + 
               static_cast<size_t>(grid_y_) * MAZE_SIZE +
               static_cast<size_t>(theta_bin_) * MAZE_SIZE * MAZE_SIZE +
               static_cast<size_t>(steering_bin_) * MAZE_SIZE * MAZE_SIZE * THETA_BINS;
    }
    
    float ComputeArcLengthFromParent(const BicycleModel& parent) const {
        if (std::fabs(steering_angle_) < 1e-2) {
            return ROBOT_SPEED * DT;
        }

        const float radius = WHEELBASE / std::tan(std::abs(steering_angle_));
        const float angular_change = std::abs(AngModPI(theta_ - parent.theta_));
        return radius * angular_change;
    }

    float DistanceTo(const BicycleModel& other) const {
        return std::sqrt((x_ - other.x_) * (x_ - other.x_) + (y_ - other.y_) * (y_ - other.y_));
    }
    
    // Getters
    float GetX() const { return x_; }
    float GetY() const { return y_; }
    float GetTheta() const { return theta_; }
    float GetSteeringAngle() const { return steering_angle_; }
    int GetGridX() const { return grid_x_; }
    int GetGridY() const { return grid_y_; }
    int GetThetaBin() const { return theta_bin_; }
    int GetSteeringBin() const { return steering_bin_; }
};
