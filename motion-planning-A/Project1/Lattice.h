#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "EnvData.h"

class Lattice {
public:
	Lattice(const ObstacleParser& obstacleParser, float resolution);
	std::vector<Eigen::Vector3f> getFreeSpacePoints() const;
	std::map<int, std::map<int, float>> getEdges() const;
	float getResolution() const;

	std::vector<Eigen::Vector3f> aStarSearch(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) const;

private:
	float resolution;
	Eigen::Vector3f lowerBounds;
	Eigen::Vector3f upperBounds;
	std::vector<Eigen::Vector3f> freeSpacePoints;
	std::map<int, std::map<int, float>> edges;
	std::vector<Obstacle> obstacles;

	void computeFreeSpacePoints();
	void buildGraph();
	bool isCollision(const Eigen::Vector3f& point) const;

	float heuristic(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) const;
	int findNearestNode(const Eigen::Vector3f& point) const;


};