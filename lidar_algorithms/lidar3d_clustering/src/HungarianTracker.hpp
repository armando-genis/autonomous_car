#ifndef HUNGARIAN_TRACKER_HPP
#define HUNGARIAN_TRACKER_HPP

#include <vector>
#include "bbox.hpp"
#include <Eigen/Dense>
#include <unordered_map>
#include <unordered_set>
#include <set>

struct Track {
    int id;
    BBox bbox;
};

class HungarianTracker {
public:
    HungarianTracker();
    ~HungarianTracker() = default;

    void obstacleTracking(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh);

private:
    std::vector<std::vector<int>> associateBoxes(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh);
    bool compareBoxes(const BBox& a, const BBox& b, float displacement_thresh, float iou_thresh);
    std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);
    bool hungarianFind(int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);
    std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);
    int searchBoxIndex(const std::vector<BBox>& boxes, int id);

    std::vector<Track> tracks_;
    int next_id_; // Next available unique ID
};

#endif // HUNGARIAN_TRACKER_HPP
