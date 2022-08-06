ALL_SRC = $(wildcard src/*.cc)
ALL_HEADER = $(wildcard src/*.h)
test: $(ALL_SRC) $(ALL_HEADER) test.cc
	g++ -O3 $(ALL_SRC) test.cc --std=c++11 -o test
main: $(ALL_SRC) $(ALL_HEADER) main.cc
	g++ -O3 $(ALL_SRC) main.cc --std=c++11 -o main
check_res: check_res.cc
	g++ -O3 check_res.cc --std=c++11 -o check_res
split: split_tunnel.cc
	g++ -O3 split_tunnel.cc --std=c++11 -o split
clean:
	rm test