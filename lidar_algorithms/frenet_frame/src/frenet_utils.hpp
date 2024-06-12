#ifndef FRENET_UTILS_HPP_
#define FRENET_UTILS_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

class QuinticPolynomial {
public:
    QuinticPolynomial(double xi, double vi, double ai, double xf, double vf, double af, double T);
    double calc_pos(double t);
    double calc_vel(double t);
    double calc_acc(double t);
    double calc_jerk(double t);

private:
    double a0, a1, a2, a3, a4, a5;
};

class QuarticPolynomial {
public:
    QuarticPolynomial(double xi, double vi, double ai, double vf, double af, double T);
    double calc_pos(double t);
    double calc_vel(double t);
    double calc_acc(double t);
    double calc_jerk(double t);

private:
    double a0, a1, a2, a3, a4;
};

class FrenetPath {
public:
    FrenetPath();

    std::vector<double> t;
    std::vector<std::vector<double>> d, s, world;
    double jd, js;
    double c_lat, c_lon, c_tot;
    double maxVelocity = 0.0, maxAcceleration = 0.0, maxCurvature = 0.0;
    double cf;
};

class FrenetUtils {
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

    std::vector<double> get_frenet(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy);
    std::vector<double> get_cartesian(double s, double d, const std::vector<double> &mapx, const std::vector<double> &mapy, const std::vector<double> &maps);

private:
    double get_dist(double x, double y, double _x, double _y);
    int get_closest_waypoints(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy);
    int next_waypoint(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy);

    double pi_ = 3.14159;
    double maxVelocity_ = 50 / 3.6; // The maximum allowed velocity for the vehicle in meters per second (m/s). Here, 50 km/h is converted to m/s.
    double maxAcceleration_ = 2;  // The maximum allowed acceleration for the vehicle in meters per second squared (m/s²).
    double maxSteeringAngle_ = 0.5; // The maximum steering angle in radians.
    double maxCurvature_ = 1; // The maximum allowable curvature for the path. Curvature is the inverse of the radius of the path.
    double robotFootprint_ = 1; // The size of the robot or vehicle's footprint in meters, typically used for collision detection.
    double maxPredictionStep_ = 5;  // The maximum step size for the prediction in the trajectory generation.
    double minPredictionStep_ = 4; // The minimum step size for the prediction in the trajectory generation.
    double noOfLanes_ = 3; // The number of lanes available for the vehicle to navigate.
    double laneWidth_ = 4;  // The width of each lane in meters.
    double targetVelocity_ = 30 / 3.6; // The target velocity for the vehicle in meters per second (m/s). Here, 30 km/h is converted to m/s.
    double velocityStep_ = 5 / 3.6; // The step size for changing the velocity in meters per second (m/s). Here, 5 km/h is converted to m/s.
    double timeStep_ = 0.1; // The time step for the trajectory generation in seconds.

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




// FrenetPath FrenetUtils::optimalTrajectory(double d0, double dv0, double da0,
//                                           double s0, double sv0,
//                                           std::vector<std::vector<double>> &centerLane,
//                                           std::vector<std::vector<double>> &obstacles,
//                                           std::vector<FrenetPath> &allPaths) {
//     std::vector<FrenetPath> paths;

//     double lateral_range = laneWidth_ / 2; // Further reduce the range for compression
//     double lateral_step = laneWidth_ / 2; // Further reduce the step size for finer paths

//     double maxPredictionStep = 4; // Reduced maximum prediction step
//     double minPredictionStep = 3; // Reduced minimum prediction step

//     for (double T = minPredictionStep; T < maxPredictionStep; T += 0.2) {
//         for (double dT = -lateral_range; dT <= lateral_range; dT += lateral_step) {
//             double dvT = 0, daT = 0;
//             std::vector<std::vector<double>> latitudinalTrajectory;
//             QuinticPolynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
//             double jd = 0;

//             for (double t = 0; t <= T; t += 0.1) {
//                 std::vector<double> data = {quintic.calc_pos(t), quintic.calc_vel(t), quintic.calc_acc(t), quintic.calc_jerk(t), t};
//                 jd += std::pow(data[3], 2);
//                 latitudinalTrajectory.push_back(data);
//             }

//             for (double svT = targetVelocity_ - velocityStep_; svT <= targetVelocity_ + velocityStep_; svT += velocityStep_) {
//                 FrenetPath path;
//                 path.t = {0.0};
//                 path.d = latitudinalTrajectory;
//                 path.jd = jd;
//                 std::vector<std::vector<double>> longitudinalTrajectory;
//                 QuarticPolynomial quartic(s0, sv0, 0, svT, 0, T);
//                 double js = 0;

//                 for (double t = 0; t <= T; t += 0.1) {
//                     std::vector<double> data = {quartic.calc_pos(t), quartic.calc_vel(t), quartic.calc_acc(t), quartic.calc_jerk(t), t};
//                     js += std::pow(data[3], 2);
//                     longitudinalTrajectory.push_back(data);
//                 }

//                 path.s = longitudinalTrajectory;
//                 path.js = js;
//                 trajectoryCost(path);
//                 paths.push_back(path);
//             }
//         }
//     }

//     convertToWorldFrame(paths, centerLane);
//     std::vector<FrenetPath> validPaths = isValid(paths, obstacles);
//     allPaths = paths;

//     if (validPaths.empty()) {
//         // RCLCPP_ERROR(rclcpp::get_logger("frenet_utils"), "No valid paths found!");
//         return FrenetPath();  // Return an empty path
//     }

//     FrenetPath optimalTrajectory;
//     double cost = std::numeric_limits<double>::max();

//     for (FrenetPath &path : validPaths) {
//         if (cost >= path.cf) {
//             cost = path.cf;
//             optimalTrajectory = path;
//         }
//     }

//     if (optimalTrajectory.d.size() <= 1 || optimalTrajectory.s.size() <= 1 || optimalTrajectory.world.size() <= 1) {
//         RCLCPP_ERROR(rclcpp::get_logger("frenet_utils"), "Optimal path has insufficient points: d.size()=%zu, s.size()=%zu, world.size()=%zu", optimalTrajectory.d.size(), optimalTrajectory.s.size(), optimalTrajectory.world.size());
//     }

//     return optimalTrajectory;
// }


