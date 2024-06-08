#include "EnvData.h"
#include "Lattice.h"
#include "trajectory.h"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
// Function to save obstacles to a CSV file
void saveObstacles(const std::vector<Obstacle>& obstacles, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "minX,maxX,minY,maxY,minZ,maxZ\n";
    for (const auto& obstacle : obstacles) {
        file << obstacle.getMinX() << "," << obstacle.getMaxX() << "," << obstacle.getMinY() << "," << obstacle.getMaxY() << "," << obstacle.getMinZ() << "," << obstacle.getMaxZ() << "\n";
    }
    file.close();
}

// Function to save lattice points to a CSV file
void saveLatticePoints(const std::vector<Eigen::Vector3f>& points, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x,y,z\n";
    for (const auto& point : points) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

// Function to save edges to a CSV file
void saveEdges(const std::vector<Eigen::Vector3f>& points, const std::map<int, std::map<int, float>>& edges, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x1,y1,z1,x2,y2,z2\n";
    for (const auto& edge : edges) {
        int point1 = edge.first;
        for (const auto& connection : edge.second) {
            int point2 = connection.first;
            file << points[point1].x() << "," << points[point1].y() << "," << points[point1].z() << ","
                << points[point2].x() << "," << points[point2].y() << "," << points[point2].z() << "\n";
        }
    }
    file.close();
}

// Function to save the path to a CSV file
void savePath(const std::vector<Eigen::Vector3f>& path, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x,y,z\n";
    for (const auto& point : path) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

// Function to save the trajectory to a CSV file
void saveTrajectory(const std::vector<Eigen::Vector3f>& trajectory, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc); 

    file << "x,y,z\n";
    for (const auto& point : trajectory) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

int main() {

	// This is a test program to test the ObstacleParser class
	ObstacleParser parser("obstacles.csv");
	std::vector<Obstacle> obstacles = parser.getObstacles();
	std::cout << "Number of obstacles: " << obstacles.size() << std::endl;
	const std::vector<float>& bounds = parser.getBounds();
	std::cout << "Bounds: " << bounds[0] << ", " << bounds[1] << ", " << bounds[2] << ", " << bounds[3] << ", " << bounds[4] << ", " << bounds[5] << std::endl;
	
	// This is a test program to test the Lattice class
	float resolutionOfLattice = 30.0;
	Lattice lattice(parser, resolutionOfLattice);
	
	// Validate the lattice construction
	std::cout << "Lattice created with resolution: " << lattice.getResolution() << std::endl;
	std::cout << "Number of free space points: " << lattice.getFreeSpacePoints().size() << std::endl;
	std::cout << "Number of edges: " << lattice.getEdges().size() << std::endl;

	// Save the obstacles and lattice points to a file
	saveObstacles(parser.getObstacles(), "obstacles_output.csv");

	// Get lattice points and edges and save them to files
	std::vector<Eigen::Vector3f> lattice_points = lattice.getFreeSpacePoints();
	std::map<int, std::map<int, float>> edges = lattice.getEdges();
	saveLatticePoints(lattice.getFreeSpacePoints(), "lattice_points_output.csv");
	saveEdges(lattice_points, edges, "edges_output.csv");

	//
	//
	// Test A* search
	//
	//

	Eigen::Vector3f start(10, 10, 10);
    Eigen::Vector3f goal(90, 90, 90);

	// Perform A* search
	std::vector<Eigen::Vector3f> path = lattice.aStarSearch(start, goal);

	// Check if path is found
	if (!path.empty()) {
		std::cout << "Path found:" << std::endl;
		for (const auto& point : path) {
			std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
		}
	}
	else {
		std::cout << "No path found." << std::endl;
	}

	// Save the path to a CSV file
	savePath(path, "path_output.csv");

    //
    //
    // Test the trajectory planner module
    //
    //
    if (!path.empty()) {
        float averageVelocity = 1.0; // Example desired average velocity
        std::vector<double> startTimes = assignSegmentStartTimes(path, averageVelocity);

        // Solve for x, y, z components separately
        Eigen::VectorXd coeffsX = solveCoefficients(path, startTimes, 'x'); // For x component
        Eigen::VectorXd coeffsY = solveCoefficients(path, startTimes, 'y'); // For y component
        Eigen::VectorXd coeffsZ = solveCoefficients(path, startTimes, 'z'); // For z component

        // Print the solutions
        std::cout << "Coefficients for x: " << coeffsX.transpose() << std::endl;
        std::cout << "Coefficients for y: " << coeffsY.transpose() << std::endl;
        std::cout << "Coefficients for z: " << coeffsZ.transpose() << std::endl;

        // Evaluate the trajectory at various time points and save the results
        std::vector<Eigen::Vector3f> trajectory;
        double totalTime = startTimes.back();
        int numSamples = 100;
        for (int i = 0; i <= numSamples; ++i) {
            double t = totalTime * i / numSamples;
            Eigen::Vector3f point = evaluateTrajectory(coeffsX, coeffsY, coeffsZ, startTimes, t);
            trajectory.push_back(point);
        }

        // Save the trajectory to a CSV file
        saveTrajectory(trajectory, "trajectory_output.csv");

        // Print the trajectory
        std::cout << "Trajectory points:" << std::endl;
        for (const auto& point : trajectory) {
            std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
        }
    }

    return 0;
}
