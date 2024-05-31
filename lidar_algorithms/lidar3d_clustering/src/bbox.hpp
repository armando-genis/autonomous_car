#ifndef BBOX_HPP
#define BBOX_HPP

struct BBox
{
    int id;  // Add this line to include an ID member
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;

    // Add additional features if needed
    float width() const { return x_max - x_min; }
    float height() const { return y_max - y_min; }
    float depth() const { return z_max - z_min; }
};

#endif // BBOX_HPP
