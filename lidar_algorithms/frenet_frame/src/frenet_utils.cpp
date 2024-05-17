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

    FrenetPath optimalTrajectory;
    double cost = INT_MAX;

    for (FrenetPath &path : validPaths) {
        if (cost >= path.cf) {
            cost = path.cf;
            optimalTrajectory = path;
        }
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
        if (!isColliding(path, obstacles) && isWithinKinematicConstraints(path)) {
            validPaths.push_back(path);
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
