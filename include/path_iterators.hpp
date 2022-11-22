// defines some iterators for following the path to the root of some nodes.

#ifndef PATH_ITERATORS_H
#define PATH_ITERATORS_H

#include "ecma.hpp"

// path calling mu,phi, alternating.
class MuPhiPath {
public:
	MuPhiPath(ECMA::Node &);

	// the same as get, but get is more comfortable to call in subclass, and
	// calling operator* from subclass is a bit annoying -> just define both
	// here.
	ECMA::Node &operator*() const;

protected:
	// not operator++ since this class should not be used as an iterator, but as
	// a superclass to an iterator.
	void advance();

	ECMA::Node &get() const;

private:
	// store current node.
	ECMA::Node *current;

	// store next function that is applied (alternate mu and phi).
	enum Function {mu, phi};
	Function next;
};

// this can be used if only the blossoms/the root, and not the entire path, is interesting.
// it will start at an outer node, and first follow rho, jumping to the base of
// its blossom (potentially itself), then follow mu (to an inner node), then
// phi (to an outer node), and then rho again.
//
// We only return base-nodes, so the current node after following rho.
class RhoMuPhiPath {
public:
	RhoMuPhiPath(ECMA::Node &);

	ECMA::Node &operator*() const;

protected:
	void advance();

	ECMA::Node &get() const;

private:
	ECMA::Node *current;
};

namespace ECMA {

// these interfaces would be nicer as templated classes using either MuPhi, or
// RhoMuPhiPath, but that's too much hassle.

class RootPath : public MuPhiPath {
public:
	using difference_type = std::ptrdiff_t;
	using value_type = Node;

	class Sentinel{};

	RootPath(Node &);

	bool operator==(const Sentinel &) const;

	// dummy int :|
	RootPath &operator++();
	void operator++(int);

private:
	void inc();
	bool one_past_root;
};

static_assert(std::input_iterator<RootPath>);
static_assert(std::sentinel_for<RootPath::Sentinel, RootPath>);

// only advances up to some specific node.
class NodePath : public MuPhiPath {
public:
	using difference_type = std::ptrdiff_t;
	using value_type = Node;

	class Sentinel{};

	// from,to respectively.
	NodePath(Node &, Node &);

	bool operator==(const Sentinel &) const;

	NodePath &operator++();
	// dummy int :|
	void operator++(int);

private:
	void inc();
	bool one_past_node;

	Node *to;
};

static_assert(std::input_iterator<NodePath>);
static_assert(std::sentinel_for<NodePath::Sentinel, NodePath>);

// advance up to the base.
class BasePath : public RhoMuPhiPath {
public:
	using difference_type = std::ptrdiff_t;
	using value_type = Node;

	class Sentinel{};

	// from,to respectively.
	BasePath(Node &);
	bool operator==(const Sentinel &) const;

	BasePath &operator++();
	// dummy int :|
	void operator++(int);

private:
	void inc();
	bool one_past_node;
};

static_assert(std::input_iterator<BasePath>);
static_assert(std::sentinel_for<BasePath::Sentinel, BasePath>);

}

#endif
