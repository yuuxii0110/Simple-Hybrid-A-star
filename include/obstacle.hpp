#pragma once

struct CircularObstacle {
    int center_x, center_y;
    int radius;
    
    CircularObstacle(int x, int y, int r) : center_x(x), center_y(y), radius(r) {}
    
    bool Contains(int x, int y) const {
        int dx = x - center_x;
        int dy = y - center_y;
        return (dx * dx + dy * dy) <= (radius * radius);
    }
};
