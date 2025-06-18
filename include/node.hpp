#pragma once

#include "bicycle_model.hpp"

struct Node {
    BicycleModel state_;
    float cost_;
    float f_score_;
    size_t parent_key_;
    
    Node(BicycleModel s = BicycleModel(), float c = 0.0f, float f = 0.0f, size_t p = SIZE_MAX) 
        : state_(s), cost_(c), f_score_(f), parent_key_(p) {}
};
