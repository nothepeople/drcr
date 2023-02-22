ALL_SRC = $(wildcard src/*.cc)
ALL_HEADER = $(wildcard src/*.h)
test: $(ALL_SRC) $(ALL_HEADER) test.cc
	g++ -O3 $(ALL_SRC) test.cc --std=c++11 -o test
main: $(ALL_SRC) $(ALL_HEADER) main.cc
	g++ -O3 $(ALL_SRC) main.cc --std=c++11 -o main
clean:
	rm test main
