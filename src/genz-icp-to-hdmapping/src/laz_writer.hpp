#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

struct Point3Di
{
    Eigen::Vector3d point;
    double timestamp;
    float intensity;
    int index_pose;
    uint8_t lidarid;
    int index_point;
};

bool saveLaz(const std::string &filename, const std::vector<Point3Di> &points_global);