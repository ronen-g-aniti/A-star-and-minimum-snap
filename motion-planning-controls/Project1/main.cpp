#include "EnvData.h"
#include "Lattice.h"
#include <iostream>

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
	
	return 0;
}