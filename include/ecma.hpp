// defines classes for the edmonds cardinality matching algorithm, and it
// itself.

#ifndef ECMA_H
#define ECMA_H

// Edmonds cardinality matching algorithm.
#include <iterator>
#include <vector>
#include <memory>
#include <set>
#include <unordered_set>
#include <fstream>

#include "graph.hpp"

namespace ECMA {

// node as used in ECMA.
// We use a node instead of tables mapping phi/mu/rho -> node since it seems a
// bit nicer (don't need to pass these tables around all the time).
// (alternative would of course be a class for ECMA-instances, but that does
// also seem a bit unwieldly, a simple function is more attractive).
class Node {
public:
	// different states a node may be in (oof =^ out-of-forest).
	enum class State {inner, outer, oof};

	// functions defined on every node, so they can just be members.
	// we can use raw pointers here, since these nodes are created+destroyed
	// together (these are not "owning" pointers).
	// these are public since the algorithm needs to be in full control of these
	// links anyway, so writing get/set-functions is superfluous.
	Node *mu;
	Node *phi;
	Node *rho;

	bool scanned;

	// still useful for visualization, maybe omit once the algorithm works.
	const ED::NodeId id;

	// constructor.
	Node(ED::NodeId);

	// I would like to delete this constructor, but then vector can't
	// contain it (it might have to be resized, and the elements moved,
	// hence requires the copy-constructor, but we will handle all vectors
	// containing Nodes in a way that won't require moving them), so instead
	// the implementation will just throw an exception to at least give some
	// runtime-safety.
	Node(Node &);
	Node(Node &&);

	// determine state of the node.
	// An enum seems better than separate outer/inner/out-of-forest-methods,
	// since it's immediately clear that enum-states are mutually exclusive.
	State state() const;

	void set_neighbors(std::vector<Node *>);

	// vector is not modifiable, but nodes are!
	const std::vector<Node *> &get_neighbors();

	// for debugging-purposes: appends the node's edges in graphviz-format.
	// PrintType determines what information is printed, and how (nocon <=> "no
	// constraint", const)
	enum class PrintType {edges, edges_invis, mu, phi, rho, mu_constraining, phi_constraining, rho_constraining};
	void print_dot(std::ostream &, std::set<PrintType>) const;

	// "neighbor-search-valid"
	// return ptr to account for no valid neighbor, as nullptr.
	Node *get_valid_neighbor();

	// need to be able to compare nodes, boils down to pointer-compare.
	bool operator==(const Node &) const;

private:
	std::vector<Node *> neighbors;
};

// just holds a list of nodes (those store the edges!) and can be converted
// to/from an EDGraph for in/output to Dimacs.
class Graph {
public:
	Graph(const ED::Graph &);
	ED::Graph matching_to_EDGraph();

	// debugging/visualization-stuff
	void print_dot(std::ostream &, std::set<Node::PrintType>);
	void capture(std::string hint = "");
	void capture_forest();
	void debug();

	// return references(pointers) to nodes, the vector may not be changed!!!
	std::vector<Node *> get_nodes();
	std::unordered_set<Node *> get_node_set();
	std::pair<std::vector<Node>::iterator, std::vector<Node>::iterator> nodes_begin_end();

private:
	// graph owns its vertices, edges are modelled inside nodes.
	std::vector<Node> nodes;

	// for captures/debugging-purposes.
	size_t capture_id;
};

// performs the edmonds cardinality-matching algorithm, and returns a ED-Graph,
// with only the matching edges.
ED::Graph ecma(const ED::Graph &g_ed);

}

#endif
