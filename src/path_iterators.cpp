#include "path_iterators.hpp"

MuPhiPath::MuPhiPath(ECMA::Node &n)
	: current{&n},
	  // apply mu next.
	  next{Function::mu} { }

void MuPhiPath::advance() {
	switch(next) {
		case Function::mu:
			current = current->mu;
			next = Function::phi;
			break;
		case Function::phi:
			current = current->phi;
			next = Function::mu;
			break;
	}
}

ECMA::Node &MuPhiPath::get() const {
	return *current;
}

ECMA::Node &MuPhiPath::operator*() const {
	return get();
}

namespace ECMA {

RootPath::RootPath(Node &n)
	: MuPhiPath(n),
	  one_past_root{false} { }

bool RootPath::operator==(const RootPath::Sentinel &) const {
	// can't check end-condition here, since we have to advance onto the root
	// once.
	return one_past_root;
}

void RootPath::inc() {
	// we're currently at the root, and now want to advance -> set one_past_root
	// now.
	// it's a bit unfortunate we have to keep track of this here, but there's no
	// other way to use c++ iterator-stuff (iterator-end <=> one-past-end,
	// phi-mu-end <=> exactly the end).
	Node &current = get();
	if (current.mu == &current && current.phi == &current)
		one_past_root = true;

	advance();
}

RootPath &RootPath::operator++() {
	inc();
	return *this;
}

void RootPath::operator++(int) {
	inc();
}

NodePath::NodePath(Node &from, Node &to)
	: MuPhiPath(from),
	  // can init with false, even if they are equal, this will be registered in
	  // the first call to ++.
	  one_past_node{false},
	  to{&to} { }

bool NodePath::operator==(const NodePath::Sentinel &) const {
	// can't check end-condition here, since we have to advance onto the root
	// once.
	return one_past_node;
}

void NodePath::inc() {
	// like in RootPath, we want to include the to-node in the path, so we
	// have to do the same trick.
	if (&get() == to) {
		one_past_node = true;
	}

	advance();
}

NodePath &NodePath::operator++() {
	inc();
	return *this;
}

void NodePath::operator++(int) {
	inc();
}

}
