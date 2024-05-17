#ifndef FRENET_UTILS_HPP_
#define FRENET_UTILS_HPP_

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

private:
    double pi_ = 3.14159;
    double maxVelocity_ = 50 / 3.6;
    double maxAcceleration_ = 2;
    double maxSteeringAngle_ = 0.7;
    double maxCurvature_ = 1;
    double robotFootprint_ = 1.5;
    double maxPredictionStep_ = 5;
    double minPredictionStep_ = 4;
    double noOfLanes_ = 5;
    double laneWidth_ = 4;
    double targetVelocity_ = 30 / 3.6;
    double velocityStep_ = 5 / 3.6;
    double timeStep_ = 0.1;
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
