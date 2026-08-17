CXX ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Wpedantic -I../firmware

.PHONY: all test clean

all: test

test: test_forward_control
	./test_forward_control

test_forward_control: test_forward_control.cpp ../firmware/forward_control.h
	$(CXX) $(CXXFLAGS) -o $@ test_forward_control.cpp

clean:
	rm -f test_forward_control
