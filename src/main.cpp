#include <fstream>
#include <iostream>

#include "graph.hpp"
#include "ecma.hpp"

int main(int argc, char **argv) {
	if (argc != 3)
	{
		std::cout << "Expected two arguments, found " << argc - 1 << std::endl;
		return EXIT_FAILURE; // return 1 would do the same, but is way too easy to mix up!
	}
	
	std::fstream input_file_graph{argv[2]};
	ED::Graph const g1 = ED::Graph::read_dimacs(input_file_graph);
	ECMA::Graph g2(g1);
	g2.print_dot(std::cout);
}
