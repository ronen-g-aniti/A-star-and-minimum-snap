#include "Lattice.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>
#include <limits>

// Define a struct to represent a node in the priority queue used in A* search
struct Node {

	int index;
	float cost;

	bool operator<(const Node& other) const {
		return cost > other.cost;
	}
};

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
	float radius = 1.0001f * resolution; // Distance between two adjacent points in the lattice, no diagonals for now; add a small buffer
	for (int i = 0; i < freeSpacePoints.size(); i++) {
		for (int j = i + 1; j < freeSpacePoints.size(); j++) {
			float distance = (freeSpacePoints[i] - freeSpacePoints[j]).norm();
			if (distance <= radius) {
				edges[i][j] = distance;
				edges[j][i] = distance;
			}
			
		}
    std::cout << "Number of edges: " << edges.size() << std::endl;
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

std::vector<Eigen::Vector3f> Lattice::getFreeSpacePoints() const {
	return freeSpacePoints;
}

std::map<int, std::map<int, float>> Lattice::getEdges() const {
	return edges;
}

float Lattice::getResolution() const {
	return resolution;
}

float Lattice::heuristic(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) const {
	return (point1 - point2).norm();
}

int Lattice::findNearestNode(const Eigen::Vector3f& point) const {
	int nearestIdx = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < freeSpacePoints.size(); ++i) {
		float dist = (freeSpacePoints[i] - point).norm();
		if (dist < minDist) {
			minDist = dist;
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

std::vector<Eigen::Vector3f> Lattice::aStarSearch(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) const {

	// Note: Nodes are indexed by their position in the freeSpacePoints vector

	// Find the nearest nodes to the start and goal points
	int startIdx = findNearestNode(start);
	int goalIdx = findNearestNode(goal);

	// Priority queue to store the nodes to be expanded
	std::priority_queue<Node> openSet;

	// Maps to store the g and f scores of each node
	std::map<int, float> gScore;
	std::map<int, float> fScore;

	// Map to store the parent of each node
	std::map<int, int> cameFrom;

	// Initialize the start point
	openSet.push({ startIdx, heuristic(start, goal) });
	gScore[startIdx] = 0;
	fScore[startIdx] = heuristic(start, goal);

	// Main loop
	while (!openSet.empty()) {

		Node currentNode = openSet.top(); // Get the lowest cost node
		openSet.pop(); // Remove the node from the open set
		int current = currentNode.index;

		// Check if the goal has been reached
		if (current == goalIdx) {
			// Reconstruct the path
			std::vector<Eigen::Vector3f> path;
			while (cameFrom.find(current) != cameFrom.end()) {
				path.push_back(freeSpacePoints[current]);
				current = cameFrom[current];
			}
			path.push_back(start);
			std::reverse(path.begin(), path.end());
			return path;
		}

		// Explore the neighbors of the current node
		for (const auto& neighbor : edges.at(current)) {
			int neighborIdx = neighbor.first;
			float tentative_gScore = gScore[current] + neighbor.second;

			// Check if this path to the neighbor is better
			if (gScore.find(neighborIdx) == gScore.end() || tentative_gScore < gScore[neighborIdx]) {
				cameFrom[neighborIdx] = current;
				gScore[neighborIdx] = tentative_gScore;
				fScore[neighborIdx] = gScore[neighborIdx] + heuristic(freeSpacePoints[neighborIdx], goal);
				openSet.push({ neighborIdx, fScore[neighborIdx] });
			}
		}
	}

	return {}; // Return an empty vector if no path is found
}
