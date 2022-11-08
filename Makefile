CXX=g++
CXXFLAGS=-pedantic -Wall -Wextra -Werror -I include -g
CXXASAN=-fsanitize=address
LDFLAGS=-g
LDASAN=-fsanitize=address
OBJECTS=build/graph.o build/main.o

./build/%.o: src/%.cpp $(HEADERS)
	$(CXX) -c -o $@ $< $(CXXFLAGS) $(CXXASAN)

# cardinality matching algorithm
CMA: $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDASAN)

run: CMA
	./CMA

check: CMA
	valgrind --leak-check=yes ./CMA

clean:
	rm -rf build/* CMA
