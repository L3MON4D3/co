// Edmonds cardinality matching algorithm.
#include <iterator>
#include <vector>
#include <memory>

#include "graph.hpp"

namespace ECMA {

// different states a node may be in (oof =^ out-of-forest).
enum NodeState {inner, outer, oof};

using distance = uint32_t;

// node as used in ECMA.
class Node {
public:
	// functions defined on every node, so they can just be members.
	// we can use raw pointers here, since these nodes are created+destroyed
	// together (these are not "owning" pointers).
	// these are public since the algorithm needs to be in full control of these
	// links anyway, so writing get/set-functions is superfluous.
	Node *mu;
	Node *phi;
	Node *rho;

	// distance from root.
	distance root_dist;

	// still useful for visualization, maybe omit once the algorithm works.
	ED::NodeId id;

	// determine state of the node.
	// An enum seems better than separate outer/inner/out-of-forest-methods,
	// since it's immediately clear that enum-states are mutually exclusive.
	NodeState state() const;

	Node(ED::NodeId);
	void set_neighbors(std::vector<Node *>);
	const std::vector<Node *> &get_neighbors() const;
	void print_dot(std::ostream &) const;

private:
	std::vector<Node *> neighbors;
};

// just holds a list of nodes (those store the edges!) and can be converted
// to/from an EDGraph for in/output to Dimacs.
class Graph {
public:
	Graph(const ED::Graph &);
	ED::Graph to_EDGraph();
	void print_dot(std::ostream &);

private:
	// graph owns its vertices, edges are modelled inside nodes.
	std::vector<Node> nodes;
};


class RootPath {
public:
	using difference_type = std::ptrdiff_t;
	using value_type = const Node;

	RootPath(Node);
	const Node &operator*() const;

	class Sentinel{};
	bool operator==(const Sentinel &) const;
	RootPath &operator++();

	// dummy int :|
	void operator++(int);

private:
	// store current node.
	Node *current;

	// store next function that is applied (alternate mu and phi).
	enum Function {mu, phi};
	Function next;
};

static_assert(std::input_iterator<RootPath>);
static_assert(std::sentinel_for<RootPath::Sentinel, RootPath>);

}
