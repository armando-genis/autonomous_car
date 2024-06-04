#include "HungarianTracker.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <set>

HungarianTracker::HungarianTracker() : next_id_(0) {}

void HungarianTracker::obstacleTracking(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh) {
    if (curr_boxes.empty() || prev_boxes.empty()) {
        // std::cerr << "Empty current or previous boxes. Exiting obstacleTracking." << std::endl;
        return;
    }

    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) {
        // std::cerr << "No connection pairs found. Exiting obstacleTracking." << std::endl;
        return;
    }

    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    matches = hungarian(connection_matrix);

    std::unordered_map<int, BBox> updated_boxes;
    std::unordered_set<int> used_ids;
    std::set<int> available_ids;

    const float movement_threshold = 0.05; // Define a threshold for significant movement

    for (const auto& box : prev_boxes) {
        used_ids.insert(box.id);
    }
    for (size_t i = 0; i < curr_boxes.size(); ++i) {
        if (used_ids.find(i) == used_ids.end()) {
            available_ids.insert(i);
        }
    }

    for (size_t j = 0; j < matches.size(); ++j) {
        const auto pre_id = pre_ids[matches[j]];
        const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
        const auto cur_id = cur_ids[j];
        const auto cur_index = searchBoxIndex(curr_boxes, cur_id);

        if (pre_index > -1 && cur_index > -1) {
            curr_boxes[cur_index].id = prev_boxes[pre_index].id;
            updated_boxes[cur_index] = curr_boxes[cur_index];

            Eigen::Vector2f prev_position_xy(prev_boxes[pre_index].position[0], prev_boxes[pre_index].position[1]);
            Eigen::Vector2f curr_position_xy(curr_boxes[cur_index].position[0], curr_boxes[cur_index].position[1]);
            Eigen::Vector2f displacement_xy = curr_position_xy - prev_position_xy;

            if (displacement_xy.norm() > movement_threshold) {
                // Update quaternion based on the new direction in the x-y plane
                Eigen::Vector3f direction(displacement_xy[0], displacement_xy[1], 0.0f);
                curr_boxes[cur_index].quaternion = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), direction);
            } else {
                // Keep the previous quaternion if movement is insignificant
                curr_boxes[cur_index].quaternion = prev_boxes[pre_index].quaternion;
            }

            used_ids.insert(prev_boxes[pre_index].id);
            available_ids.erase(prev_boxes[pre_index].id);
            // std::cout << "Assigned ID: " << prev_boxes[pre_index].id << " to current box at index: " << cur_index << std::endl;
        }
    }

    next_id_ = std::min(static_cast<int>(curr_boxes.size()) + 7, next_id_);

    for (size_t i = 0; i < curr_boxes.size(); ++i) {
        if (updated_boxes.find(i) == updated_boxes.end()) {
            int new_id;
            if (!available_ids.empty()) {
                new_id = *available_ids.begin();
                available_ids.erase(available_ids.begin());
            } else {
                new_id = next_id_++;
            }
            curr_boxes[i].id = new_id;
            // Calculate quaternion based on the direction from the origin (0,0,0) to the current position in the x-y plane
            Eigen::Vector2f curr_position_xy(curr_boxes[i].position[0], curr_boxes[i].position[1]);
            Eigen::Vector3f direction(curr_position_xy[0], curr_position_xy[1], 0.0f);
            curr_boxes[i].quaternion = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitX(), direction);
            updated_boxes[i] = curr_boxes[i];
            used_ids.insert(new_id);
        }
    }

    // std::cout << "Completed obstacleTracking function." << std::endl;
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

std::vector<std::vector<int>> HungarianTracker::associateBoxes(const std::vector<BBox>& prev_boxes, std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh) {
    std::vector<std::vector<int>> connection_pairs;

    for (size_t i = 0; i < prev_boxes.size(); ++i) {
        for (size_t j = 0; j < curr_boxes.size(); ++j) {
            if (compareBoxes(prev_boxes[i], curr_boxes[j], displacement_thresh, iou_thresh)) {
                if (curr_boxes[j].id == -1) {
                    curr_boxes[j].id = next_id_++;
                    // std::cout << "Assigned new ID to current box: " << curr_boxes[j].id << " at index: " << j << std::endl;
                }
                connection_pairs.emplace_back(std::initializer_list<int>{prev_boxes[i].id, curr_boxes[j].id});
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
    for (size_t j = 0; j < connection_matrix[i].size(); ++j) {
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
    for (size_t i = 0; i < connection_matrix.size(); ++i) {
        std::fill(right_connected.begin(), right_connected.end(), false);
        if (hungarianFind(i, connection_matrix, right_connected, right_pair)) {
            count++;
        }
    }

    std::vector<int> result;
    for (size_t i = 0; i < right_pair.size(); ++i) {
        if (right_pair[i] != -1) {
            result.push_back(right_pair[i]);
        }
    }
    return result;
}

int HungarianTracker::searchBoxIndex(const std::vector<BBox>& boxes, int id) {
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].id == id) {
            return i;
        }
    }
    return -1;
}
