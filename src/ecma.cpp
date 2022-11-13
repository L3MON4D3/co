#include "ecma.hpp"
#include <iostream>
#include <memory>

namespace ECMA {

Node::Node(ED::NodeId id)
	// initialize with pointer to self.
	: mu{this},
	  phi{this},
	  rho{this},
	  // after initialization, the node is the root of its own tree.
	  root_dist{0},
	  id{id},
	  neighbors{} {}

NodeState Node::state() const {
	if (mu == this || mu->phi != this)
		return NodeState::outer;
	else if (mu->phi == phi && phi != this)
		return NodeState::inner;
	else
		return NodeState::oof;
}

void Node::set_neighbors(std::vector<Node *> new_neighbors) {
	neighbors = new_neighbors;
}


const std::vector<Node *> &Node::get_neighbors() const { return neighbors; }

void Node::print_dot(std::ostream &s) const {
	// only print edges where the neighbors id is higher, that way duplicated
	// edges are prevented.
	// (edge is only printed in one partners `print_dot`)
	for (auto n : neighbors)
		if (id < n->id)
			// +1: match dimacs-description.
			s << "\t" << id+1 << " -- " << n->id+1 << std::endl;
}

Graph::Graph(const ED::Graph &G)
	: nodes{} {

	// `nodes` maps G's node-ids to Nodes.
	// this isn't that nice, but it works as long as the node-ids cover
	// exactly the range 0-G.num_nodes(), which is the case for now.
	// resize default-initializes the nodes.
	//
	// important!! `nodes` may not be changed after this (no new elements added).
	// We have raw pointers to elements inside this vector.
	nodes.reserve(G.num_nodes());
	for (unsigned int i = 0; i != G.num_nodes(); ++i)
		// will never move vector!
		nodes.push_back(Node(i));

	// with all nodes present, we can construct the neighbor-lists
	// (they require raw pointers).
	//
	// iterate node-ids, and create a neighbor-list
	for (uint32_t i = 0; i != G.num_nodes(); ++i) {
		std::vector<Node *> neighbors;
		for (ED::NodeId neighbor_id : G.node(i).neighbors())
			neighbors.push_back(&nodes[neighbor_id]);
		nodes[i].set_neighbors(neighbors);
	}
}

ED::Graph Graph::to_EDGraph() {
	// construct empty EDGraph.
	ED::Graph G(nodes.size());

	// insert edges.
	for (auto &v : nodes)
		for (auto &n : v.get_neighbors()) {
			G.add_edge(v.id, n->id);
		}

	return G;
}

void Graph::print_dot(std::ostream &s) {
	s << "graph {" << std::endl;

	for (auto node : nodes)
		node.print_dot(s);

	s << "}" << std::endl;
}

}
