/* obstacle_detector.hpp

 * Copyright (C) 2021 SS47816

 * Implementation of 3D LiDAR Obstacle Detection & Tracking Algorithms

**/

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "box.hpp"

namespace lidar_obstacle_detector
{
template <typename PointT>
class ObstacleDetector
{
 public:
  ObstacleDetector();
  virtual ~ObstacleDetector();

  // ****************** Detection ***********************
  
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clustering_v2(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size, const bool use_height);



  // ****************** Tracking ***********************
  void obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);

 private:
  // ****************** Detection ***********************
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

  // ****************** Tracking ***********************
  bool compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh);

  // Link nearby bounding boxes between the previous and previous frame
  std::vector<std::vector<int>> associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh);

  // Connection Matrix
  std::vector<std::vector<int>> connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right);

  // Helper function for Hungarian Algorithm
  bool hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair);

  // Customized Hungarian Algorithm
  std::vector<int> hungarian(const std::vector<std::vector<int>>& connection_matrix);

  // Helper function for searching the box index in boxes given an id
  int searchBoxIndex(const std::vector<Box>& Boxes, const int id);
};

// constructor:
template <typename PointT>
ObstacleDetector<PointT>::ObstacleDetector() {}

// de-constructor:
template <typename PointT>
ObstacleDetector<PointT>::~ObstacleDetector() {}



template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::separateClouds(const pcl::PointIndices::ConstPtr& inliers, const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());

  // Pushback all the inliers into the ground_cloud
  for (int index : inliers->indices)
  {
    ground_cloud->points.push_back(cloud->points[index]);
  }

  // Extract the points that are not in the inliers to obstacle_cloud
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstacle_cloud, ground_cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::clustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size)
{
  // Time clustering process
  // const auto start_time = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (auto& getIndices : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    for (auto& index : getIndices.indices)
      cluster->points.push_back(cloud->points[index]);

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    clusters.push_back(cluster);
  }

  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}



template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ObstacleDetector<PointT>::clustering_v2(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const float cluster_tolerance, const int min_size, const int max_size, const bool use_height)
{
  // Time clustering process
  // const auto start_time = std::chrono::steady_clock::now();

  // Optionally convert to 2D point cloud
  typename pcl::PointCloud<PointT>::Ptr converted_cloud(new pcl::PointCloud<PointT>);
  if (!use_height) {
      for (const auto& point : cloud->points) {
          PointT point2d;
          point2d.x = point.x;
          point2d.y = point.y;
          point2d.z = 0.0; // Set z to 0 to project into 2D
          converted_cloud->push_back(point2d);
      }
  } else {
      converted_cloud = std::const_pointer_cast<pcl::PointCloud<PointT>>(cloud);
  }

  // Create KdTree for clustering
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(converted_cloud);

  // Perform Euclidean clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(converted_cloud);
  ec.extract(cluster_indices);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  for (const auto& getIndices : cluster_indices) {
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
      for (auto& index : getIndices.indices) {
          cluster->points.push_back(converted_cloud->points[index]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
  }



  // const auto end_time = std::chrono::steady_clock::now();
  // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  // std::cout << "clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

// ************************* Tracking ***************************
template <typename PointT>
void ObstacleDetector<PointT>::obstacleTracking(const std::vector<Box>& prev_boxes, std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  // Tracking (based on the change in size and displacement between frames)
  
  if (curr_boxes.empty() || prev_boxes.empty())
  {
    return;
  }
  else
  {
    // vectors containing the id of boxes in left and right sets
    std::vector<int> pre_ids;
    std::vector<int> cur_ids;
    std::vector<int> matches;

    // Associate Boxes that are similar in two frames
    auto connection_pairs = associateBoxes(prev_boxes, curr_boxes, displacement_thresh, iou_thresh);

    if (connection_pairs.empty()) return;

    // Construct the connection matrix for Hungarian Algorithm's use
    auto connection_matrix = connectionMatrix(connection_pairs, pre_ids, cur_ids);

    // Use Hungarian Algorithm to solve for max-matching
    matches = hungarian(connection_matrix);

    for (int j = 0; j < matches.size(); ++j)
    {
      // find the index of the previous box that the current box corresponds to
      const auto pre_id = pre_ids[matches[j]];
      const auto pre_index = searchBoxIndex(prev_boxes, pre_id);
      
      // find the index of the current box that needs to be changed
      const auto cur_id = cur_ids[j]; // right and matches has the same size
      const auto cur_index = searchBoxIndex(curr_boxes, cur_id);
      
      if (pre_index > -1 && cur_index > -1)
      {
        // change the id of the current box to the same as the previous box
        curr_boxes[cur_index].id = prev_boxes[pre_index].id;
      }
    }
  }
}

template <typename PointT>
bool ObstacleDetector<PointT>::compareBoxes(const Box& a, const Box& b, const float displacement_thresh, const float iou_thresh)
{
  // Percetage Displacements ranging between [0.0, +oo]
  const float dis = sqrt((a.position[0] - b.position[0]) * (a.position[0] - b.position[0]) + (a.position[1] - b.position[1]) * (a.position[1] - b.position[1]) + (a.position[2] - b.position[2]) * (a.position[2] - b.position[2]));

  const float a_max_dim = std::max(a.dimension[0], std::max(a.dimension[1], a.dimension[2]));
  const float b_max_dim = std::max(b.dimension[0], std::max(b.dimension[1], b.dimension[2]));
  const float ctr_dis = dis / std::min(a_max_dim, b_max_dim);

  // Dimension similiarity values between [0.0, 1.0]
  const float x_dim = 2 * (a.dimension[0] - b.dimension[0]) / (a.dimension[0] + b.dimension[0]);
  const float y_dim = 2 * (a.dimension[1] - b.dimension[1]) / (a.dimension[1] + b.dimension[1]);
  const float z_dim = 2 * (a.dimension[2] - b.dimension[2]) / (a.dimension[2] + b.dimension[2]);

  if (ctr_dis <= displacement_thresh && x_dim <= iou_thresh && y_dim <= iou_thresh && z_dim <= iou_thresh)
  {
    return true;
  }
  else
  {
    return false;
  }
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::associateBoxes(const std::vector<Box>& prev_boxes, const std::vector<Box>& curr_boxes, const float displacement_thresh, const float iou_thresh)
{
  std::vector<std::vector<int>> connection_pairs;

  for (auto& prev_box : prev_boxes)
  {
    for (auto& curBox : curr_boxes)
    {
      // Add the indecies of a pair of similiar boxes to the matrix
      if (this->compareBoxes(curBox, prev_box, displacement_thresh, iou_thresh))
      {
        connection_pairs.push_back({prev_box.id, curBox.id});
      }
    }
  }

  return connection_pairs;
}

template <typename PointT>
std::vector<std::vector<int>> ObstacleDetector<PointT>::connectionMatrix(const std::vector<std::vector<int>>& connection_pairs, std::vector<int>& left, std::vector<int>& right)
{
  // Hash the box ids in the connection_pairs to two vectors(sets), left and right
  for (auto& pair : connection_pairs)
  {
    bool left_found = false;
    for (auto i : left)
    {
      if (i == pair[0])
        left_found = true;
    }
    if (!left_found)
      left.push_back(pair[0]);

    bool right_found = false;
    for (auto j : right)
    {
      if (j == pair[1])
        right_found = true;
    }
    if (!right_found)
      right.push_back(pair[1]);
  }

  std::vector<std::vector<int>> connection_matrix(left.size(), std::vector<int>(right.size(), 0));

  for (auto& pair : connection_pairs)
  {
    int left_index = -1;
    for (int i = 0; i < left.size(); ++i)
    {
      if (pair[0] == left[i])
        left_index = i;
    }

    int right_index = -1;
    for (int i = 0; i < right.size(); ++i)
    {
      if (pair[1] == right[i])
        right_index = i;
    }

    if (left_index != -1 && right_index != -1)
      connection_matrix[left_index][right_index] = 1;
  }

  return connection_matrix;
}

template <typename PointT>
bool ObstacleDetector<PointT>::hungarianFind(const int i, const std::vector<std::vector<int>>& connection_matrix, std::vector<bool>& right_connected, std::vector<int>& right_pair)
{
  for (int j = 0; j < connection_matrix[0].size(); ++j)
  {
    if (connection_matrix[i][j] == 1 && right_connected[j] == false)
    {
      right_connected[j] = true;

      if (right_pair[j] == -1 || hungarianFind(right_pair[j], connection_matrix, right_connected, right_pair))
      {
        right_pair[j] = i;
        return true;
      }
    }
  }
}

template <typename PointT>
std::vector<int> ObstacleDetector<PointT>::hungarian(const std::vector<std::vector<int>>& connection_matrix)
{
  std::vector<bool> right_connected(connection_matrix[0].size(), false);
  std::vector<int> right_pair(connection_matrix[0].size(), -1);

  int count = 0;
  for (int i = 0; i < connection_matrix.size(); ++i)
  {
    if (hungarianFind(i, connection_matrix, right_connected, right_pair))
      count++;
  }

  std::cout << "For: " << right_pair.size() << " current frame bounding boxes, found: " << count << " matches in previous frame! " << std::endl;

  return right_pair;
}

template <typename PointT>
int ObstacleDetector<PointT>::searchBoxIndex(const std::vector<Box>& boxes, const int id)
{
  for (int i = 0; i < boxes.size(); i++)
  {
    if (boxes[i].id == id)
    return i;
  }

  return -1;
}

} // namespace lidar_obstacle_detector