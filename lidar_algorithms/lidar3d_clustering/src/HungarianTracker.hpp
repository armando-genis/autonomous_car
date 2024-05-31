#ifndef HUNGARIAN_TRACKER_HPP
#define HUNGARIAN_TRACKER_HPP

#include "bbox.hpp"
#include <Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>

struct Track {
    int id;
    Eigen::Vector3f centroid;
    BBox bbox;
};

class HungarianTracker {
public:
    HungarianTracker(float displacement_thresh, float iou_thresh)
        : displacement_thresh_(displacement_thresh), iou_thresh_(iou_thresh), next_id_(0) {}

    void update(const std::vector<BBox>& prev_boxes, const std::vector<Eigen::Vector3f>& prev_centroids, const std::vector<Eigen::Vector3f>& curr_centroids, const std::vector<BBox>& curr_boxes);
    const std::vector<Track>& getTracks() const { return tracks_; }

private:
    std::vector<Track> tracks_;
    float displacement_thresh_;
    float iou_thresh_;
    int next_id_; // Next available unique ID

    std::vector<std::vector<int>> associateBoxes(const std::vector<BBox>& prev_boxes, const std::vector<BBox>& curr_boxes, const std::vector<Eigen::Vector3f>& prev_centroids, const std::vector<Eigen::Vector3f>& curr_centroids, float displacement_thresh, float iou_thresh);
    bool compareBoxes(const BBox& a, const BBox& b, const Eigen::Vector3f& centroid_a, const Eigen::Vector3f& centroid_b, float displacement_thresh, float iou_thresh);
    std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);
    bool hungarianFind(int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);
    std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);
    int searchBoxIndex(const std::vector<BBox>& boxes, int id);
};

#endif // HUNGARIAN_TRACKER_HPP
