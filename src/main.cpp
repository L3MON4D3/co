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
	const ED::Graph g1 = ED::Graph::read_dimacs(input_file_graph);
	ECMA::ecma(g1);
}
