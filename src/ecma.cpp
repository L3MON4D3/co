#include "ecma.hpp"
#include "path_iterators.hpp"

#include <algorithm>
#include <bits/ranges_base.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <ranges>
#include <stack>

#include <cassert>
#include <sstream>
#include <filesystem>

namespace ECMA {

Node::Node(ED::NodeId id)
	// initialize functions with self.
	: mu{this},
	  phi{this},
	  rho{this},
	  scanned{false},
	  id{id},
	  neighbors{} {}

Node::Node(Node &)
	// we just want to fail, but id needs to be initialized...
	: id{0} {

	// reasoning in declaration.
	throw;
}

Node::Node(Node &&)
	: id{0} {
	throw;
}

NodeState Node::state() const {
	if (mu == this || mu->phi != mu) {
		return NodeState::outer;
	} else if (mu->phi == mu && phi != this) {
		return NodeState::inner;
	} else {
		assert(mu != this && phi == this && mu->phi == mu);
		return NodeState::oof;
	}
}

void Node::set_neighbors(std::vector<Node *> new_neighbors) {
	neighbors = new_neighbors;
}

const std::vector<Node *> &Node::get_neighbors() const { return neighbors; }

void Node::print_dot(std::ostream &s, std::set<PrintType> options) const {
	if (options.contains(PrintType::edges) || options.contains(PrintType::edges_invis)) {

		std::string style;
		if (options.contains(PrintType::edges))
			style = " [color = black]";
		else
			style = " [color = black, style=invis]";

		// only print edges where the neighbors id is higher, that way duplicated
		// edges are prevented.
		// (edge is only printed in one partners `print_dot`)
		for (auto n : neighbors)
			// if (id < n->id)
				// black arrows for regular edges.
				s << "\t" << id << " -> " << n->id << style << std::endl;
	}

	// set constraint false so the graph always looks the same in different
	// debug-steps.
	if (options.contains(PrintType::mu))
		s << "\t" << id << " -> " << mu->id << " [color = red, constraint = false]" << std::endl;
	if (options.contains(PrintType::phi))
		s << "\t" << id << " -> " << phi->id << " [color = blue, constraint = false]" << std::endl;
	if (options.contains(PrintType::rho))
		s << "\t" << id << " -> " << rho->id << " [color = green, constraint = false]" << std::endl;
	if (options.contains(PrintType::mu_nocon))
		s << "\t" << id << " -> " << mu->id << " [color = red]" << std::endl;
	if (options.contains(PrintType::phi_nocon))
		s << "\t" << id << " -> " << phi->id << " [color = blue]" << std::endl;
	if (options.contains(PrintType::rho_nocon))
		s << "\t" << id << " -> " << rho->id << " [color = green]" << std::endl;
}

Node *Node::get_valid_neighbor() {
	for (Node *n : neighbors) {
		NodeState n_state = n->state();
		if (n_state == NodeState::oof || (n_state == NodeState::outer && n->rho != rho))
			return n;
	}

	return nullptr;
}

bool Node::operator==(const Node &n) const {
	// only one instance of each node.
	return this == &n;
}


Graph::Graph(const ED::Graph &G)
	: nodes{},
	  capture_id{0} {

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
		// careful: use emplace_back to construct the Node in-place.
		nodes.emplace_back(i);

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

void Graph::print_dot(std::ostream &s, std::set<Node::PrintType> p) {
	s << "digraph {" << std::endl;

	for (auto &node : nodes)
		node.print_dot(s, p);

	s << "}" << std::endl;
}

void Graph::debug() {
	std::fstream deb("debug.dot");
	print_dot(deb, {Node::PrintType::mu, Node::PrintType::edges_invis, Node::PrintType::phi, Node::PrintType::rho});
	deb.flush();
}

void Graph::capture(std::string hint) {
	// open file for this capture.
	std::stringstream s;
	s << "./captures/" << std::setfill('0') << std::setw(5) << capture_id << "_" << hint << ".dot";
	std::ofstream capture(s.str());

	using P = Node::PrintType;
	print_dot(capture, {P::mu, P::edges_invis, P::phi, P::rho});

	// in case we crash..
	capture.flush();

	++capture_id;
}

void Graph::capture_forest() {
	std::ofstream capture("forest.dot");
	capture << "digraph {" << std::endl;

	using P = Node::PrintType;
	for (auto &node : nodes)
		if (node.state() == NodeState::inner || node.state() == NodeState::outer)
			node.print_dot(capture, {P::edges, P::mu_nocon, P::phi_nocon, P::rho_nocon});

	capture << "}" << std::endl;
}

std::vector<Node *> Graph::get_nodes() {
	// prepare+fill vector.
	std::vector<Node *> ns;
	ns.reserve(nodes.size());
	for (Node &n : nodes)
		ns.push_back(&n);

	return ns;
}

std::unordered_set<Node *> Graph::get_node_set() {
	// prepare+fill vector.
	std::unordered_set<Node *> ns;
	for (Node &n : nodes)
		ns.insert(&n);

	return ns;
}

std::pair<std::vector<Node>::iterator, std::vector<Node>::iterator> Graph::nodes_begin_end() {
	return {nodes.begin(), nodes.end()};
}


void reset(std::unordered_set<Node *> &set) {
	for (Node *_v : set) {
		Node &v = *_v;
		v.phi = &v;
		v.rho = &v;
		v.scanned = false;
	}
}

std::unordered_set<Node *> tree(Node &root) {
	std::unordered_set<Node *> visited;
	std::stack<Node *> to_visit;
	to_visit.push(&root);

	// we know that from every node in the tree, there is a mu-phi-path to the
	// root. We can thus find the tree, by following only edges where mu/phi
	// points towards the current node.
	while (!to_visit.empty()) {
		Node *v = to_visit.top();
		to_visit.pop();

		for (Node *w: v->get_neighbors())
			if ((w->mu == v || w->phi == v) && !visited.contains(w)) {
				visited.insert(w);
				to_visit.push(w);
			}
	}

	return visited;
}

Node &root(Node &x) {
	BasePath px = BasePath(x);
	std::ranges::advance(px, BasePath::Sentinel());
	
	return *px;
}

void invert_path(RootPath p) {
	std::ranges::advance(p, 1);

	while (p != RootPath::Sentinel()) {
		// store current node, advance, and _then_ change phi,mu.
		// (unchanged phi,mu are neede for advancing properly.)
		Node &current = *p;
		std::ranges::advance(p, 2);
		// phi matches, or phi is a root.
		current.phi->mu = &current;
		current.mu = current.phi;
	}
}

Node &first_common_base(Node &x, Node &y) {
	std::vector<Node *> base_on_px;
	for (BasePath px(x); px != BasePath::Sentinel(); ++px) {
		base_on_px.push_back(&*px);
	}

	std::vector<Node *> base_on_py;
	for (BasePath py(y); py != BasePath::Sentinel(); ++py) {
		base_on_py.push_back(&*py);
	}

	// there is at least one common base, we want to find the first, so the one
	// before which the bases differ for px and py.
	// iterate from back, store first where 
	auto i_by = base_on_py.rbegin();
	auto i_bx = base_on_px.rbegin();

	// important: check rend-equality first, otherwise we'll have bad access at
	// *i_by and *i_bx.
	while (i_by != base_on_py.rend() && i_bx != base_on_px.rend() && *i_by == *i_bx) {
		++i_by;
		++i_bx;
	}
	assert(*(i_by-1) == *(i_bx-1));

	// first star to get pointer from iterator, second to get value from pointer
	// (and return it as reference).
	return **(--i_by);
}

void shrink(Node &x, Node &y, Node &r) {
	// adjust ear-decomposition.
	NodePath pxr(x, r);
	// odd nodes only!
	std::ranges::advance(pxr, 1);
	while (pxr != NodePath::Sentinel()) {
		// store current node, advance, and _then_ change phi.
		Node &current = *pxr;
		std::ranges::advance(pxr, 2);
		if (current.phi->rho != &r)
			current.phi->phi = &current;
	}

	NodePath pyr(y, r);
	std::ranges::advance(pyr, 1);
	while (pyr != NodePath::Sentinel()) {
		Node &current = *pyr;
		std::ranges::advance(pyr, 2);
		if (current.phi->rho != &r)
			current.phi->phi = &current;
	}

	if (x.rho != &r)
		x.phi = &y;
	if (y.rho != &r)
		y.phi = &x;
}

void ecma(const ED::Graph &g_ed) {
	// construct ECMA-graph.
	Graph g(g_ed);

	// store nodes here too, we might have to access them quite often.
	std::unordered_set<Node *> all_nodes = g.get_node_set();

	// the nodes we will actually work on.
	std::unordered_set<Node *> effective_nodes = all_nodes;

	// potential outer, unscanned nodes, will be covered in next iteration.
	// (use set to avoid inserting a node twice)
	// We have to append:
	// - the new outer node in grow
	// - (at least) the trees of x and y in augment
	// - (at least) the new blossom in shrink (since there might be new inner
	// nodes, that became outer).
	std::unordered_set<Node *> next_nodes;

	// jump back here if we haven't handled all nodes yet.
	redo:
	for (Node *_x : effective_nodes) {
		Node &x = *_x;
		// need outer vertex.
		if (x.state() != NodeState::outer || x.scanned) {
			continue;
		}

		for (Node *_y : x.get_neighbors()) {
			Node &y = *_y;
			NodeState y_state = y.state();
			if (y_state == NodeState::oof) {
				// grow
				y.phi = &x;

				// check new outer node in next iteration.
				next_nodes.insert(y.mu);

				// g.capture("grow");

				continue;
			} else if (y_state == NodeState::outer && y.rho != x.rho) {
				Node &x_root = root(x);
				Node &y_root = root(y);
				if (x_root != y_root) {
					// store trees for reset.
					std::unordered_set x_tree = tree(x_root);
					std::unordered_set y_tree = tree(y_root);

					// augment
					invert_path(RootPath(x));
					invert_path(RootPath(y));
					x.mu = &y;
					y.mu = &x;

					// reset(all_nodes);
					// next_nodes = all_nodes;

					reset(x_tree);
					reset(y_tree);
					next_nodes.merge(x_tree);
					next_nodes.merge(y_tree);

					// also re-scan all neighbors of the trees.
					for (Node *v : x_tree)
						for (Node *w: v->get_neighbors()) {
							w->scanned = false;
							next_nodes.insert(w);
						}
					for (Node *v : y_tree)
						for (Node *w: v->get_neighbors()) {
							w->scanned = false;
							next_nodes.insert(w);
						}

					// g.capture("augment");
					goto continue_vertex_scan;
				} else {
					Node &r = first_common_base(x, y);

					// shrink
					shrink(x,y,r);
					
					// set rho for all nodes in the blossom.

					// create hashmap of all nodes on [x,r] and [y,r] (to
					// quickly query whether rho(v) is on them, and should now
					// point to r)
					std::unordered_set<Node *> xr_yr_nodes;
					for (NodePath pxr(x,r); pxr != NodePath::Sentinel(); ++pxr)
						// get iterator element, then get its address.
						xr_yr_nodes.insert(&(*pxr));
					for (NodePath pyr(y,r); pyr != NodePath::Sentinel(); ++pyr)
						xr_yr_nodes.insert(&(*pyr));

					auto [shrink_nodes_iter, shrink_nodes_end] = g.nodes_begin_end();
					for (; shrink_nodes_iter != shrink_nodes_end; ++shrink_nodes_iter) {
						Node &v = *shrink_nodes_iter;
						if (xr_yr_nodes.contains(v.rho)) {
							v.rho = &r;

							// all these nodes now belong to an outer blossom,
							// whereas they might have been inner vertices,
							// before.
							// Look at them in the next iteration.
							next_nodes.insert(&v);
						}
					}

					// g.capture("shrink");
				}
			}
		}
		// we can neither grow, shrink, or augment at x,
		// continue with next unscanned vertex.
		x.scanned = true;

		continue_vertex_scan:;
	}

	if (!next_nodes.empty()) {
		// if there are still nodes to check, restart loop.
		swap(effective_nodes, next_nodes);
		// clear next nodes for next loop.
		next_nodes.clear();
		goto redo;
	}

	// assert there are no leftover outer, unscanned nodes.
	for (Node *n : all_nodes)
		assert((n->state()==NodeState::outer && n->scanned==true) || n->state() != NodeState::outer);

	g.capture_forest();

	// print graphviz to stdout.
	// g.print_dot(std::cout, {Node::PrintType::edges, Node::PrintType::mu});

	// find size of matching by subtracting #exposed vertices from #vertices and
	// dividing by 2
	size_t exposed_n = 0;
	for (Node *n : g.get_nodes())
		if (n->mu == n)
			++exposed_n;
	std::cout << (g.get_nodes().size()-exposed_n)/2 << std::endl;
	std::cout << exposed_n << std::endl;
}

}
