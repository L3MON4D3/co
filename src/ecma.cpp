#include "ecma.hpp"
#include "path_iterators.hpp"

#include <algorithm>
#include <bits/ranges_base.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <ranges>

#include <cassert>
#include <sstream>
#include <filesystem>

#include <unordered_map>

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

std::pair<std::vector<Node>::iterator, std::vector<Node>::iterator> Graph::nodes_iters() {
	return {nodes.begin(), nodes.end()};
}


bool same_root(Node &x, Node &y) {
	// walk both to root and compare.
	// TODO: use rho for jumping to root faster.
	RootPath px = RootPath(x);
	std::ranges::advance(px, RootPath::Sentinel());

	RootPath py = RootPath(y);
	std::ranges::advance(py, RootPath::Sentinel());

	return *px == *py;
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
	for (RootPath px(x); px != RootPath::Sentinel(); ++px) {
		Node &r = *px;
		if (r.rho == &r)
			base_on_px.push_back(&r);
	}

	std::vector<Node *> base_on_py;
	for (RootPath py(y); py != RootPath::Sentinel(); ++py) {
		Node &r = *py;
		if (r.rho == &r)
			base_on_py.push_back(&r);
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

	auto [nodes_begin, nodes_end] = g.nodes_iters();
	auto nodes_iter = nodes_begin;

	while (nodes_iter != nodes_end) {
		Node &x = *nodes_iter;

		// need outer vertex.
		if (x.state() != NodeState::outer || x.scanned) {
			++nodes_iter;
			continue;
		}

		for (Node *_y : x.get_neighbors()) {
			Node &y = *_y;
			NodeState y_state = y.state();
			if (y_state == NodeState::oof) {
				// grow
				y.phi = &x;
				// std::cout << "grow" << std::endl;
				// g.capture("grow");
				nodes_iter = nodes_begin;
				continue;
			} else if (y_state == NodeState::outer && y.rho != x.rho) {
				if (!same_root(x, y)) {
					// augment
					invert_path(RootPath(x));
					invert_path(RootPath(y));
					x.mu = &y;
					y.mu = &x;

					// improve this!!
					nodes_iter = nodes_begin;

					auto [reset_iter, reset_end] = g.nodes_iters();
					for (; reset_iter != reset_end; ++reset_iter) {
						Node &v = *reset_iter;
						v.phi = &v;
						v.rho = &v;
						v.scanned = false;
					}
					// std::cout << "augment" << std::endl;
					// g.capture("augment");
					// continue outer loop.
					goto continue_vertex_scan;
				} else {
					Node &r = first_common_base(x, y);

					// shrink
					shrink(x,y,r);
					
					// create hashmap of all nodes on [x,r] and [y,r]
					std::unordered_map<Node *, bool> xr_yr_nodes;
					for (NodePath pxr(x,r); pxr != NodePath::Sentinel(); ++pxr)
						// get iterator element, then get its address.
						xr_yr_nodes.insert({&(*pxr), true});
					for (NodePath pyr(y,r); pyr != NodePath::Sentinel(); ++pyr)
						xr_yr_nodes.insert({&(*pyr), true});

					auto [shrink_nodes_iter, shrink_nodes_end] = g.nodes_iters();
					for (; shrink_nodes_iter != shrink_nodes_end; ++shrink_nodes_iter) {
						Node &v = *shrink_nodes_iter;
						if (xr_yr_nodes.contains(v.rho))
							v.rho = &r;
					}

					// std::cout << "shrink" << std::endl;
					// g.capture("shrink");
					nodes_iter = nodes_begin;
				}
			}
		}
		// we can neither grow, shrink, or augment at x,
		// continue with next unscanned vertex.
		x.scanned = true;
		++nodes_iter;

		continue_vertex_scan:;
	}

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
