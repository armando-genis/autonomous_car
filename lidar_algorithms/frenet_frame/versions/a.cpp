#include "frenet_utils.hpp"

FrenetUtils::FrenetUtils() {
    // Initialization if needed
}

FrenetUtils::~FrenetUtils() {
}

FrenetPath FrenetUtils::optimalTrajectory(double d0, double dv0, double da0,
                                          double s0, double sv0,
                                          std::vector<std::vector<double>> &centerLane,
                                          std::vector<std::vector<double>> &obstacles,
                                          std::vector<FrenetPath> &allPaths) {
    std::vector<FrenetPath> paths;

    for (double T = minPredictionStep_; T < maxPredictionStep_; T += 0.2) {
        for (double dT = -((noOfLanes_ - 1) * laneWidth_) / 2; dT <= ((noOfLanes_ - 1) * laneWidth_) / 2; dT += laneWidth_) {
            double dvT = 0, daT = 0;
            std::vector<std::vector<double>> latitudinalTrajectory;
            Polynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
            double jd = 0;

            for (double t = 0; t <= T; t += 0.1) {
                std::vector<double> data = {quintic.position(t), quintic.velocity(t), quintic.acceleration(t), quintic.jerk(t), t};
                jd += std::pow(data[3], 2);
                latitudinalTrajectory.push_back(data);
            }

            for (double svT = targetVelocity_ - velocityStep_; svT <= targetVelocity_ + velocityStep_; svT += velocityStep_) {
                FrenetPath path;
                path.T = T;
                path.d = latitudinalTrajectory;
                path.jd = jd;
                std::vector<std::vector<double>> longitudinalTrajectory;
                Polynomial quartic(s0, sv0, 0, svT, 0, T);
                double js = 0;

                for (double t = 0; t <= T; t += 0.1) {
                    std::vector<double> data = {quartic.position(t), quartic.velocity(t), quartic.acceleration(t), quartic.jerk(t), t};
                    js += std::pow(data[3], 2);
                    if (data[1] > path.maxVelocity)
                        path.maxVelocity = data[1];
                    if (data[2] > path.maxAcceleration)
                        path.maxAcceleration = data[2];
                    longitudinalTrajectory.push_back(data);
                }

                path.s = longitudinalTrajectory;
                path.js = js;
                trajectoryCost(path);
                paths.push_back(path);
            }
        }
    }

    convertToWorldFrame(paths, centerLane);
    std::vector<FrenetPath> validPaths = isValid(paths, obstacles);
    allPaths = paths;

    if (validPaths.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("frenet_utils"), "No valid paths found!");
        return FrenetPath();  // Return an empty path
    }

    FrenetPath optimalTrajectory;
    double cost = INT_MAX;

    for (FrenetPath &path : validPaths) {
        if (cost >= path.cf) {
            cost = path.cf;
            optimalTrajectory = path;
        }
    }

    if (optimalTrajectory.d.size() <= 1 || optimalTrajectory.s.size() <= 1 || optimalTrajectory.world.size() <= 1) {
        RCLCPP_ERROR(rclcpp::get_logger("frenet_utils"), "Optimal path has insufficient points: d.size()=%zu, s.size()=%zu, world.size()=%zu", optimalTrajectory.d.size(), optimalTrajectory.s.size(), optimalTrajectory.world.size());
    }

    return optimalTrajectory;
}

void FrenetUtils::trajectoryCost(FrenetPath &path) {
    double cd = path.jd * kjd_ + path.T * ktd_ + std::pow(path.d.back()[0], 2) * ksd_;
    double cv = path.js * kjs_ + path.T * kts_ + std::pow(path.s.front()[0] - path.s.back()[0], 2) * kss_;
    path.cf = klat_ * cd + klon_ * cv;
}

bool FrenetUtils::isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles) {
    for (size_t i = 0; i < path.world.size(); ++i) {
        for (size_t j = 0; j < obstacles.size(); ++j) {
            if (std::sqrt(std::pow(path.world[i][0] - obstacles[j][0], 2) + std::pow(path.world[i][1] - obstacles[j][1], 2)) <= 3)
                return true;
        }
    }
    return false;
}

bool FrenetUtils::isWithinKinematicConstraints(FrenetPath &path) {
    if (path.maxVelocity > maxVelocity_ || path.maxAcceleration > maxAcceleration_ || path.maxCurvature > maxCurvature_) {
        return false;
    }
    return true;
}

std::vector<FrenetPath> FrenetUtils::isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles) {
    std::vector<FrenetPath> validPaths;
    for (FrenetPath &path : paths) {
        bool colliding = isColliding(path, obstacles);
        bool withinConstraints = isWithinKinematicConstraints(path);

        if (!colliding && withinConstraints) {
            validPaths.push_back(path);
            RCLCPP_INFO(rclcpp::get_logger("frenet_utils"), "\033[1;32m----> VALID PATH\033[0m");

        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("frenet_utils"), "Path invalidated. Colliding: %s, Within Constraints: %s",
                         colliding ? "true" : "false", withinConstraints ? "true" : "false");

            RCLCPP_INFO(rclcpp::get_logger("frenet_utils"), "\033[1;36m----> NOT VALID PATH\033[0m");
            
        }
    }
    return validPaths;
}

void FrenetUtils::convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane) {
    for (FrenetPath &path : paths) {
        size_t j = 0;
        for (size_t i = 0; i < path.s.size(); ++i) {
            double x, y, yaw;
            for (; j < centerLane.size(); ++j) {
                if (std::abs(path.s[i][0] - centerLane[j][4]) <= 0.1) {
                    x = centerLane[j][0];
                    y = centerLane[j][1];
                    yaw = centerLane[j][2];
                    break;
                }
            }
            double d = path.d[i][0];
            path.world.push_back({x + d * std::cos(yaw + 1.57), y + d * std::sin(yaw + 1.57), 0, 0});
        }

        for (size_t i = 0; i < path.world.size() - 1; ++i) {
            path.world[i][2] = std::atan2((path.world[i + 1][1] - path.world[i][1]), (path.world[i + 1][0] - path.world[i][0]));
            path.world[i][3] = std::sqrt(std::pow(path.world[i + 1][0] - path.world[i][0], 2) + std::pow(path.world[i + 1][1] - path.world[i][1], 2));
        }
        path.world[path.world.size() - 1][2] = path.world[path.world.size() - 2][2];
        path.world[path.world.size() - 1][3] = path.world[path.world.size() - 2][3];

        double curvature = INT_MIN;
        for (size_t i = 0; i < path.world.size() - 1; ++i) {
            double yaw_diff = path.world[i + 1][2] - path.world[i][2];
            if (curvature <= std::abs(yaw_diff / path.world[i][3]))
                curvature = std::abs(yaw_diff / path.world[i][3]);
        }
        path.maxCurvature = curvature;
    }
}




#ifndef FRENET_UTILS_HPP_
#define FRENET_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>


#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "polynomial.hpp"
#include "frenetPath.hpp"

class FrenetUtils
{
public:
    FrenetUtils();
    ~FrenetUtils();

    FrenetPath optimalTrajectory(double d0, double dv0, double da0,
                                 double s0, double sv0,
                                 std::vector<std::vector<double>> &centerLane,
                                 std::vector<std::vector<double>> &obstacles,
                                 std::vector<FrenetPath> &allPaths);

    bool isColliding(FrenetPath &path, std::vector<std::vector<double>> &obstacles);
    void convertToWorldFrame(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &centerLane);
    void trajectoryCost(FrenetPath &path);
    bool isWithinKinematicConstraints(FrenetPath &path);
    std::vector<FrenetPath> isValid(std::vector<FrenetPath> &paths, std::vector<std::vector<double>> &obstacles);

// base_footprint

private:
    double pi_ = 3.14159;
    double maxVelocity_ = 50 / 3.6; //The maximum allowed velocity for the vehicle in meters per second (m/s). Here, 50 km/h is converted to m/s.

    double maxAcceleration_ = 2;  //The maximum allowed acceleration for the vehicle in meters per second squared (m/s²).
    double maxSteeringAngle_ = 0.5; //The maximum steering angle in radians.
    double maxCurvature_ = 1; //The maximum allowable curvature for the path. Curvature is the inverse of the radius of the path.
    double robotFootprint_ = 1; //The size of the robot or vehicle's footprint in meters, typically used for collision detection.
    double maxPredictionStep_ = 5;  //The maximum step size for the prediction in the trajectory generation.
    double minPredictionStep_ = 4; //The minimum step size for the prediction in the trajectory generation.
    double noOfLanes_ = 15; //The number of lanes available for the vehicle to navigate.
    double laneWidth_ = 4;  //The width of each lane in meters.
    double targetVelocity_ = 30 / 3.6; //The target velocity for the vehicle in meters per second (m/s). Here, 30 km/h is converted to m/s.
    double velocityStep_ = 5 / 3.6; //The step size for changing the velocity in meters per second (m/s). Here, 5 km/h is converted to m/s.
    double timeStep_ = 0.1; //The time step for the trajectory generation in seconds.


    double klat_ = 1; 
    double klon_ = 1;
    double kjd_ = 0.1;
    double ktd_ = 0.1;
    double ksd_ = 2;
    double kjs_ = 0.1;
    double kts_ = 0.1;
    double kss_ = 2;
};

#endif // FRENET_UTILS_HPP_ 

