#ifndef HUNGARIAN_TRACKER_HPP
#define HUNGARIAN_TRACKER_HPP

#include <vector>
#include "bbox.hpp"

class HungarianTracker {
public:
    HungarianTracker() = default;
    ~HungarianTracker() = default;

    void obstacleTracking(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh);

private:
    bool compareBoxes(const BBox& a, const BBox& b, float displacement_thresh, float iou_thresh);
    std::vector<std::vector<int>> associateBoxes(const std::vector<BBox>& prev_boxes, const std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh);
    std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);
    bool hungarianFind(int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);
    std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);
    int searchBoxIndex(const std::vector<BBox>& boxes, int id);
};

#endif // HUNGARIAN_TRACKER_HPP
