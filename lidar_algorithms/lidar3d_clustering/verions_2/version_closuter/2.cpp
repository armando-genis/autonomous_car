#include "HungarianTracker.hpp"
#include <cmath>
#include <iostream>

void HungarianTracker::obstacleTracking(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh) {
    if (curr_boxes.empty() || prev_boxes.empty()) {
        return;
    }

    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j) {
        const auto pre_id = pre_ids[matches[j]];
        const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
        const auto cur_id = cur_ids[j];
        const auto cur_index = searchBoxIndex(curr_boxes, cur_id);

        if (pre_index > -1 && cur_index > -1) {
            curr_boxes[cur_index].id = prev_boxes[pre_index].id;
        }
    }
}

bool HungarianTracker::compareBoxes(const BBox& a, const BBox& b, float displacement_thresh, float iou_thresh) {
    const float dis = sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) + 
                           (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) + 
                           (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

    const float a_max_dim = std::max({a.dimension[0], a.dimension[1], a.dimension[2]});
    const float b_max_dim = std::max({b.dimension[0], b.dimension[1], b.dimension[2]});
    const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

    const float x_dim = 2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
    const float y_dim = 2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
    const float z_dim = 2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

    return (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh);
}

std::vector<std::vector<int>> HungarianTracker::associateBoxes(const std::vector<BBox>& prev_boxes, const std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh) {
    std::vector<std::vector<int>> connection_pairs;

    for (const auto& prev_box : prev_boxes) {
        for (const auto& curBox : curr_boxes) {
            if (compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh)) {
                connection_pairs.push_back({prev_box.id, curBox.id});
            }
        }
    }

    return connection_pairs;
}

std::vector<std::vector<int>> HungarianTracker::connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right) {
    for (const auto& pair : connection_pairs) {
        if (std::find(left.begin(), left.end(), pair[0]) == left.end()) {
            left.push_back(pair[0]);
        }
        if (std::find(right.begin(), right.end(), pair[1]) == right.end()) {
            right.push_back(pair[1]);
        }
    }

    std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

    for (const auto& pair : connection_pairs) {
        auto left_index = std::find(left.begin(), left.end(), pair[0]) - left.begin();
        auto right_index = std::find(right.begin(), right.end(), pair[1]) - right.begin();
        connection_matrix[left_index][right_index] = 1;
    }

    return connection_matrix;
}

bool HungarianTracker::hungarianFind(int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair) {
    for (int j = 0; j < connection_matrix[i].size(); ++j) {
        if (connection_matrix[i][j] == 1 && !right_connected[j]) {
            right_connected[j] = true;

            if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair)) {
                right_pair[j] = i;
                return true;
            }
        }
    }
    return false;
}

std::vector<int> HungarianTracker::hungarian(const std::vector<std::vector<int>>& connection_matrix) {
    std::vector<bool> right_connected(connection_matrix[0].size(), false);
    std::vector<int> right_pair(connection_matrix[0].size(), -1);

    int count = 0;
    for (int i = 0; i < connection_matrix.size(); ++i) {
        std::fill(right_connected.begin(), right_connected.end(), false);
        if (hungarianFind(i, connection_matrix, right_connected, right_pair)) {
            ++count;
        }
    }

    std::cout << "For: " << right_pair.size() << " current frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

    return right_pair;
}

int HungarianTracker::searchBoxIndex(const std::vector<BBox>& boxes, int id) {
    for (int i = 0; i < boxes.size(); i++) {
        if (boxes[i].id == id)
            return i;
    }
    return -1;
}
