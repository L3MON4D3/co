CXX=g++ -std=c++20
CXXFLAGS=-O2 -pedantic -Wall -Wextra -Werror -I include
DEBUG=-DNDEBUG
# uncomment to generate graphviz of each step in "./captures".
CAPTURE=#-DCAPTURE_STEPS
CXXASAN=#-fsanitize=address

LDFLAGS=#-g
LDASAN=#-fsanitize=address
OBJECTS=build/graph.o build/main.o build/ecma.o build/path_iterators.o
HEADERS=$(wildcard include/*.hpp)

./build/%.o: src/%.cpp Makefile ${HEADERS}
	${CXX} -c -o $@ $< ${CXXFLAGS} ${CXXASAN} ${DEBUG} ${CAPTURE}

# cardinality matching algorithm
CMA: ${OBJECTS} ${HEADERS}
	${CXX} -o $@ $^ ${LDFLAGS} ${LDASAN}

run: CMA
	./CMA

check: CMA
	valgrind --leak-check=yes ./CMA

clean:
	rm -rf build/* CMA

capture_convert:
	cd captures && for f in *; do dot -Tsvg "$$f" -O ; done
clean_captures:
	rm -rf captures/*
