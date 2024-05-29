#include "frenet_utils.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

// QuinticPolynomial implementation
QuinticPolynomial::QuinticPolynomial(double xi, double vi, double ai, double xf, double vf, double af, double T) {
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;

    Eigen::Matrix3d A;
    Eigen::Vector3d b, x;

    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T, 5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;

    b << xf - (a0 + a1*T + a2*T*T),
         vf - (a1 + 2*a2*T),
         af - (2*a2);

    x = A.colPivHouseholderQr().solve(b);

    a3 = x[0];
    a4 = x[1];
    a5 = x[2];
}

double QuinticPolynomial::calc_pos(double t) {
    return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
}

double QuinticPolynomial::calc_vel(double t) {
    return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
}

double QuinticPolynomial::calc_acc(double t) {
    return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
}

double QuinticPolynomial::calc_jerk(double t) {
    return 6*a3 + 24*a4*t + 60*a5*t*t;
}

// QuarticPolynomial implementation
QuarticPolynomial::QuarticPolynomial(double xi, double vi, double ai, double vf, double af, double T) {
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;

    Eigen::Matrix2d A;
    Eigen::Vector2d b, x;

    A << 3*T*T, 4*T*T*T,
         6*T, 12*T*T;

    b << vf - (a1 + 2*a2*T),
         af - (2*a2);

    x = A.colPivHouseholderQr().solve(b);

    a3 = x[0];
    a4 = x[1];
}

double QuarticPolynomial::calc_pos(double t) {
    return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t;
}

double QuarticPolynomial::calc_vel(double t) {
    return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t;
}

double QuarticPolynomial::calc_acc(double t) {
    return 2*a2 + 6*a3*t + 12*a4*t*t;
}

double QuarticPolynomial::calc_jerk(double t) {
    return 6*a3 + 24*a4*t;
}

// FrenetPath implementation
FrenetPath::FrenetPath() : jd(0), js(0), c_lat(0), c_lon(0), c_tot(0), cf(0) {}

// FrenetUtils implementation
FrenetUtils::FrenetUtils() {}

FrenetUtils::~FrenetUtils() {}

double FrenetUtils::get_dist(double x, double y, double _x, double _y) {
    return std::sqrt((x - _x) * (x - _x) + (y - _y) * (y - _y));
}

int FrenetUtils::get_closest_waypoints(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy) {
    double closestLen = std::numeric_limits<double>::max(); //large number
    int closestWaypoint = 0;

    for (int i = 0; i < mapx.size(); i++) {
        double map_x = mapx[i];
        double map_y = mapy[i];
        double dist = get_dist(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int FrenetUtils::next_waypoint(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy) {
    int closestWaypoint = get_closest_waypoints(x, y, mapx, mapy);

    double map_x = mapx[closestWaypoint];
    double map_y = mapy[closestWaypoint];

    double heading = std::atan2((map_y - y), (map_x - x));
    double angle = std::abs(std::atan2(y, x) - heading);

    if (angle > (pi_ / 4)) {
        closestWaypoint++;
    }
    return closestWaypoint;
}

std::vector<double> FrenetUtils::get_frenet(double x, double y, const std::vector<double> &mapx, const std::vector<double> &mapy) {
    int next_wp = next_waypoint(x, y, mapx, mapy);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = mapx.size() - 1;
    }

    double n_x = mapx[next_wp] - mapx[prev_wp];
    double n_y = mapy[next_wp] - mapy[prev_wp];
    double x_x = x - mapx[prev_wp];
    double x_y = y - mapy[prev_wp];

    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_y + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = get_dist(x_x, x_y, proj_x, proj_y);

    double ego_vec_x = x - mapx[prev_wp];
    double ego_vec_y = y - mapy[prev_wp];
    double map_vec_x = n_x;
    double map_vec_y = n_y;
    double d_cross = ego_vec_x * map_vec_y - ego_vec_y * map_vec_x;

    if (d_cross > 0) {
        frenet_d = -frenet_d;
    }

    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += get_dist(mapx[i], mapy[i], mapx[i + 1], mapy[i + 1]);
    }
    frenet_s += get_dist(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

std::vector<double> FrenetUtils::get_cartesian(double s, double d, const std::vector<double> &mapx, const std::vector<double> &mapy, const std::vector<double> &maps) {
    int prev_wp = -1;

    while (s > maps[prev_wp + 1] && (prev_wp < (int)(maps.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % mapx.size();

    double heading = std::atan2((mapy[wp2] - mapy[prev_wp]), (mapx[wp2] - mapx[prev_wp]));
    double seg_s = (s - maps[prev_wp]);

    double seg_x = mapx[prev_wp] + seg_s * std::cos(heading);
    double seg_y = mapy[prev_wp] + seg_s * std::cos(heading);

    double perp_heading = heading - pi_ / 2;

    double x = seg_x + d * std::cos(perp_heading);
    double y = seg_y + d * std::sin(perp_heading);

    return {x, y};
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
            QuinticPolynomial quintic(d0, dv0, da0, dT, dvT, daT, T);
            double jd = 0;

            for (double t = 0; t <= T; t += 0.1) {
                std::vector<double> data = {quintic.calc_pos(t), quintic.calc_vel(t), quintic.calc_acc(t), quintic.calc_jerk(t), t};
                jd += std::pow(data[3], 2);
                latitudinalTrajectory.push_back(data);
            }

            for (double svT = targetVelocity_ - velocityStep_; svT <= targetVelocity_ + velocityStep_; svT += velocityStep_) {
                FrenetPath path;
                path.t = {0.0};
                path.d = latitudinalTrajectory;
                path.jd = jd;
                std::vector<std::vector<double>> longitudinalTrajectory;
                QuarticPolynomial quartic(s0, sv0, 0, svT, 0, T);
                double js = 0;

                for (double t = 0; t <= T; t += 0.1) {
                    std::vector<double> data = {quartic.calc_pos(t), quartic.calc_vel(t), quartic.calc_acc(t), quartic.calc_jerk(t), t};
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
    double cost = std::numeric_limits<double>::max();

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
    double cd = path.jd * kjd_ + path.t.back() * ktd_ + std::pow(path.d.back()[0], 2) * ksd_;
    double cv = path.js * kjs_ + path.t.back() * kts_ + std::pow(path.s.front()[0] - path.s.back()[0], 2) * kss_;
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
            path.world.push_back({x + d * std::cos(yaw + M_PI / 2), y + d * std::sin(yaw + M_PI / 2), 0, 0});
        }

        for (size_t i = 0; i < path.world.size() - 1; ++i) {
            path.world[i][2] = std::atan2((path.world[i + 1][1] - path.world[i][1]), (path.world[i + 1][0] - path.world[i][0]));
            path.world[i][3] = std::sqrt(std::pow(path.world[i + 1][0] - path.world[i][0], 2) + std::pow(path.world[i + 1][1] - path.world[i][1], 2));
        }
        path.world[path.world.size() - 1][2] = path.world[path.world.size() - 2][2];
        path.world[path.world.size() - 1][3] = path.world[path.world.size() - 2][3];

        double curvature = std::numeric_limits<double>::min();
        for (size_t i = 0; i < path.world.size() - 1; ++i) {
            double yaw_diff = path.world[i + 1][2] - path.world[i][2];
            if (curvature <= std::abs(yaw_diff / path.world[i][3]))
                curvature = std::abs(yaw_diff / path.world[i][3]);
        }
        path.maxCurvature = curvature;
    }
}
