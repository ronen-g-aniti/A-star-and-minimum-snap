#pragma once

#include <vector>
#include <Eigen/Dense>

double calculateDistance(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2);
std::vector<double> assignSegmentStartTimes(const std::vector<Eigen::Vector3f>& waypoints, const float averageVelocity);
Eigen::VectorXd solveCoefficients(const std::vector<Eigen::Vector3f>& waypoints, const std::vector<double>& startTimes, char component);
std::vector<double> normalizeStartTimes(const std::vector<double>& startTimes);
Eigen::Vector3f evaluateTrajectory(const Eigen::VectorXd& coeffsX, const Eigen::VectorXd& coeffsY, const Eigen::VectorXd& coeffsZ, const std::vector<double>& startTimes, double t);
