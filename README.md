Run `make` for building, `./CMA --graphs <somegraph>` to run the algorithm,
output is the matching in dimacs-format.

Make sure gcc-12.2 (others might also work, just 11 does not work) is installed.

It's possible to generate graphviz-visualizations for every step of the
algorithm, enable in Makefile (`-DCAPTURE_STEPS`) (not recommended for graphs
bigger than about 100 nodes :D), they can be converted to svg via make-target
`capture_convert`.

overview:
* `path_iterators.hpp` defines iterators for walking the mu-phi-paths (or
  rho-mu-phi)
* `graph.hpp` is the graph-class from the website, we only use it for converting
  DIMACS to a graph-structure, our algorithm works on the graph-class in
  `ecma.hpp`
* `ecma.hpp` contains the node and graph specifically for the
  matching-algorithm
