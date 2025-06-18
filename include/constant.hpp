#pragma once

#include <cmath>

const int MAZE_SIZE = 100;
const float CELL_SIZE = 1.0;
const float WHEELBASE = 3.32;
const float MAX_STEERING_ANGLE = 35.0f * M_PI / 180.0f;
const float MAX_STEERING_RATE = 25.0f * M_PI / 180.0f;
const float ROBOT_SPEED = 1.0f;

const int THETA_BINS = 20;
const int STEERING_BINS = 11;

const float MIN_DISTANCE_PER_STEP = 1.414f;
const float MIN_STEERING_PER_STEP = (2 * MAX_STEERING_ANGLE) / (STEERING_BINS * MAX_STEERING_RATE);
const float DT = std::max(MIN_DISTANCE_PER_STEP / ROBOT_SPEED, MIN_STEERING_PER_STEP / MAX_STEERING_RATE);

const int MIN_OBSTACLES = 5;
const int MAX_OBSTACLES = 10;
const int MIN_OBSTACLE_RADIUS = 2;
const int MAX_OBSTACLE_RADIUS = 10;
const int CLEAR_RADIUS = 10;

const float M_PI_F = static_cast<float>(M_PI);
