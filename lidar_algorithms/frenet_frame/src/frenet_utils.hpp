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

