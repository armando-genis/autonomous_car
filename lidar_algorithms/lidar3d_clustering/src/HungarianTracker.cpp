#include "HungarianTracker.hpp"
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

void HungarianTracker::update(const std::vector<BBox>& prev_boxes, const std::vector<Eigen::Vector3f>& centroids, const std::vector<BBox>& curr_boxes) {
    if (prev_boxes.empty() || curr_boxes.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "No previous or current boxes to track.");
        return;
    }

    std::vector<int> pre_ids, cur_ids;
    std::vector<std::vector<int>> connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh_, iou_thresh_);

    if (connection_pairs.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "No connection pairs found.");
        return;
    }

    std::vector<std::vector<int>> connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);
    std::vector<int> matches = hungarian(connection_matrix);

    std::unordered_map<int, Track> updated_tracks;
    std::unordered_set<int> used_ids;

    // Update existing tracks based on matches
    for (int j = 0; j < matches.size(); ++j) {
        int pre_id = pre_ids[matches[j]];
        int cur_id = cur_ids[j];
        int cur_index = searchBoxIndex(curr_boxes, cur_id);

        RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Match found: prev_id=%d, cur_id=%d, cur_index=%d", pre_id, cur_id, cur_index);

        if (cur_index >= 0 && cur_index < curr_boxes.size()) {
            if (pre_id < tracks_.size()) {
                updated_tracks[cur_index] = Track{tracks_[pre_id].id, centroids[cur_index], curr_boxes[cur_index]};
                used_ids.insert(tracks_[pre_id].id);
                RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Track updated: id=%d", updated_tracks[cur_index].id);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("HungarianTracker"), "Invalid track index: pre_id=%d, tracks_.size()=%zu", pre_id, tracks_.size());
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("HungarianTracker"), "Invalid index: cur_index=%d", cur_index);
        }
    }

    // Assign new IDs to unmatched current boxes
    for (int i = 0; i < curr_boxes.size(); ++i) {
        if (updated_tracks.find(i) == updated_tracks.end()) { // Assign new ID
            int new_id = next_id_;
            while (used_ids.find(new_id) != used_ids.end()) {
                new_id = next_id_++;
            }
            updated_tracks[i] = Track{new_id, centroids[i], curr_boxes[i]};
            used_ids.insert(new_id);
            RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "New track created: id=%d", updated_tracks[i].id);
        }
    }

    // Move the updated tracks to the main tracks list
    tracks_.clear();
    for (auto& [_, track] : updated_tracks) {
        tracks_.push_back(track);
    }

    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Updated %zu tracks", tracks_.size());
}

std::vector<std::vector<int>> HungarianTracker::associateBoxes(const std::vector<BBox>& prev_boxes, const std::vector<BBox>& curr_boxes, float displacement_thresh, float iou_thresh) {
    std::vector<std::vector<int>> connection_pairs;
    for (size_t i = 0; i < prev_boxes.size(); ++i) {
        for (size_t j = 0; j < curr_boxes.size(); ++j) {
            if (compareBoxes(prev_boxes[i], curr_boxes[j], displacement_thresh, iou_thresh)) {
                connection_pairs.emplace_back(std::initializer_list<int>{static_cast<int>(i), static_cast<int>(j)});
            }
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Found %zu connection pairs", connection_pairs.size());
    return connection_pairs;
}

bool HungarianTracker::compareBoxes(const BBox& a, const BBox& b, float displacement_thresh, float iou_thresh) {
    float dis = std::sqrt(std::pow(a.x_min - b.x_min, 2) + std::pow(a.y_min - b.y_min, 2) + std::pow(a.z_min - b.z_min, 2));
    float a_max_dim = std::max({a.x_max - a.x_min, a.y_max - a.y_min, a.z_max - a.z_min});
    float b_max_dim = std::max({b.x_max - b.x_min, b.y_max - b.y_max, b.z_max - b.z_max});
    float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

    if (ctr_dis > displacement_thresh) return false;

    float x_dim = 2 * (a.x_max - b.x_max) / (a.x_max + b.x_max);
    float y_dim = 2 * (a.y_max - b.y_max) / (a.y_max + b.y_max);
    float z_dim = 2 * (a.z_max - b.z_max) / (a.z_max + b.z_max);

    return (x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh);
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
        int left_index = std::distance(left.begin(), std::find(left.begin(), left.end(), pair[0]));
        int right_index = std::distance(right.begin(), std::find(right.begin(), right.end(), pair[1]));
        connection_matrix[left_index][right_index] = 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Constructed connection matrix with size %zu x %zu", connection_matrix.size(), connection_matrix[0].size());
    return connection_matrix;
}

bool HungarianTracker::hungarianFind(int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair) {
    for (int j = 0; j < connection_matrix[0].size(); ++j) {
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
            count++;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Hungarian algorithm found %d matches", count);
    return right_pair;
}

int HungarianTracker::searchBoxIndex(const std::vector<BBox>& boxes, int id) {
    if (id >= 0 && id < boxes.size()) {
        return id;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("HungarianTracker"), "searchBoxIndex: id %d is out of bounds (boxes size: %zu)", id, boxes.size());
        return -1;
    }
}
