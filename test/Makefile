CXX ?= g++
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Wpedantic -I../CVCC_Controller

.PHONY: all test clean

all: test

test: test_control
	./test_control

test_control: test_control.cpp ../CVCC_Controller/CVCCController.h \
              ../CVCC_Controller/PIController.h ../CVCC_Controller/util.h \
              ../CVCC_Controller/config.h
	$(CXX) $(CXXFLAGS) -o $@ test_control.cpp

clean:
	rm -f test_control
