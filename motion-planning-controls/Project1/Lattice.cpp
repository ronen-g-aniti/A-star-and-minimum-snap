#include "Lattice.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

Lattice::Lattice(const ObstacleParser& obstacleParser, float resolution)
	: resolution(resolution), obstacles(obstacleParser.getObstacles()) {
	lowerBounds = Eigen::Vector3f(obstacleParser.getBounds()[0], obstacleParser.getBounds()[2], obstacleParser.getBounds()[4]);
	upperBounds = Eigen::Vector3f(obstacleParser.getBounds()[1], obstacleParser.getBounds()[3], obstacleParser.getBounds()[5]);
	computeFreeSpacePoints();
	buildGraph();
}

void Lattice::computeFreeSpacePoints() {
	for (float x = lowerBounds.x(); x <= upperBounds.x(); x += resolution) {
		for (float y = lowerBounds.y(); y <= upperBounds.y(); y += resolution) {
			for (float z = lowerBounds.z(); z <= upperBounds.z(); z += resolution) {
				Eigen::Vector3f point(x, y, z);
				if (!isCollision(point)) {
					freeSpacePoints.push_back(point);
				}
			}
		}
	}
}

void Lattice::buildGraph() {
	float radius = 1.0f * resolution; // Distance between two adjacent points in the lattice, no diagonals for now
	for (int i = 0; i < freeSpacePoints.size(); i++) {
		for (int j = i + 1; j < freeSpacePoints.size(); j++) {
			if (!isCollisionBetweenPoints(freeSpacePoints[i], freeSpacePoints[j])) {
				float distance = (freeSpacePoints[i] - freeSpacePoints[j]).norm();
				if (distance <= radius) {
					edges[i][j] = distance;
					edges[j][i] = distance;
					std::cout << "Edge between " << i << " and " << j << " with distance " << distance << std::endl;
				}
			}
		}
	}
}

bool Lattice::isCollision(const Eigen::Vector3f& point) const {
	for (const Obstacle& obstacle : obstacles) {
		if (obstacle.isCollision(point)) {
			return true;
		}
	}
	return false;
}

bool Lattice::isCollisionBetweenPoints(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) const {
	Eigen::Vector3f direction = point2 - point1;
	float length = direction.norm();
	direction.normalize();


	// Vectorized collision check

	// 1. Create a set of points along the line between point1 and point2
	int num_steps = static_cast<int>(length);
	Eigen::Vector3f step = direction; // Step size is 1.0 meter
	Eigen::MatrixXf points = Eigen::MatrixXf::Zero(3, num_steps + 1);
	for (int i = 0; i <= num_steps; ++i) {
		points.col(i) = point1 + step * i;
	}

	// 2. Check if any of the points are in collision
	for (int i = 0; i <= num_steps; ++i) {
		if (isCollision(points.col(i))) {
			return true;
		}
	}
	return false;
}

std::vector<Eigen::Vector3f> Lattice::getFreeSpacePoints() const {
	return freeSpacePoints;
}

std::map<int, std::map<int, float>> Lattice::getEdges() const {
	return edges;
}

float Lattice::getResolution() const {
	return resolution;
}