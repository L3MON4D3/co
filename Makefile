CXX=g++ -std=c++20
CXXFLAGS= -pedantic -Wall -Wextra -Werror -I include -g -fconcepts-diagnostics-depth=4
CXXASAN=-fsanitize=address

LDFLAGS=-g
LDASAN=-fsanitize=address
OBJECTS=build/graph.o build/main.o build/ecma.o
HEADERS=$(wildcard include/*.hpp)

./build/%.o: src/%.cpp Makefile $(HEADERS)
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(CXXASAN)

# cardinality matching algorithm
CMA: $(OBJECTS) $(HEADERS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDASAN)

run: CMA
	./CMA

check: CMA
	valgrind --leak-check=yes ./CMA

clean:
	rm -rf build/* CMA
