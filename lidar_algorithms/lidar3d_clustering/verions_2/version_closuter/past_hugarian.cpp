#include "HungarianTracker.hpp"
#include "bbox.hpp"
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>

void HungarianTracker::update(const std::vector<BBox>& prev_boxes, const std::vector<Eigen::Vector3f>& prev_centroids, const std::vector<Eigen::Vector3f>& curr_centroids, const std::vector<BBox>& curr_boxes) {
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
    std::set<int> available_ids;

    // Collect used IDs and generate a set of available IDs up to the number of clusters detected
    for (const auto& track : tracks_) {
        used_ids.insert(track.id);
    }
    for (int i = 0; i < curr_boxes.size(); ++i) {
        if (used_ids.find(i) == used_ids.end()) {
            available_ids.insert(i);
        }
    }

    // Update existing tracks based on matches
    for (int j = 0; j < matches.size(); ++j) {
        int pre_id = pre_ids[j];
        int cur_id = cur_ids[matches[j]];
        int cur_index = searchBoxIndex(curr_boxes, cur_id);

        RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Match found: prev_id=%d, cur_id=%d, cur_index=%d", pre_id, cur_id, cur_index);

        if (cur_index >= 0 && cur_index < curr_boxes.size()) {
            if (pre_id < tracks_.size()) {
                updated_tracks[cur_index] = Track{tracks_[pre_id].id, curr_centroids[cur_index], curr_boxes[cur_index]};
                used_ids.insert(tracks_[pre_id].id);
                available_ids.erase(tracks_[pre_id].id);
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
            int new_id;
            if (!available_ids.empty()) {
                new_id = *available_ids.begin();
                available_ids.erase(available_ids.begin());
            } else {
                // Generate a new ID if no available IDs left
                new_id = used_ids.size();
            }
            updated_tracks[i] = Track{new_id, curr_centroids[i], curr_boxes[i]};
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
    // Calculate Euclidean distance between the centers of the bounding boxes
    const float dis = std::sqrt(
        std::pow(a.centerX() - b.centerX(), 2) +
        std::pow(a.centerY() - b.centerY(), 2) +
        std::pow(a.centerZ() - b.centerZ(), 2)
    );

    // Calculate the maximum dimension of both bounding boxes
    const float a_max_dim = std::max({a.width(), a.height(), a.depth()});
    const float b_max_dim = std::max({b.width(), b.height(), b.depth()});

    // Calculate the normalized center distance
    const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

    // Calculate the dimension similarity values
    const float x_dim = 2 * std::abs(a.width() - b.width()) / (a.width() + b.width());
    const float y_dim = 2 * std::abs(a.height() - b.height()) / (a.height() + b.height());
    const float z_dim = 2 * std::abs(a.depth() - b.depth()) / (a.depth() + b.depth());

    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Comparing boxes: displacement=%.2f, iou=%.2f", ctr_dis, z_dim);

    // Check if the bounding boxes are similar based on the thresholds
    return (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh);
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
        int i = std::find(left.begin(), left.end(), pair[0]) - left.begin();
        int j = std::find(right.begin(), right.end(), pair[1]) - right.begin();
        connection_matrix[i][j] = 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("HungarianTracker"), "Created connection matrix: left_size=%zu, right_size=%zu", left.size(), right.size());
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
    std::vector<int> right_pair(connection_matrix[0].size(), -1);

    for (size_t i = 0; i < connection_matrix.size(); ++i) {
        std::vector<bool> right_connected(connection_matrix[0].size(), false);
        hungarianFind(i, connection_matrix, right_connected, right_pair);
    }

    std::vector<int> match(connection_matrix.size(), -1);
    for (size_t j = 0; j < right_pair.size(); ++j) {
        if (right_pair[j] != -1) {
            match[right_pair[j]] = j;
        }
    }
    return match;
}

int HungarianTracker::searchBoxIndex(const std::vector<BBox>& boxes, int id) {
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (boxes[i].id == id) {
            return i;
        }
    }
    return -1;
}
