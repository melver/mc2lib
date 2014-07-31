
CXX = g++
CXXFLAGS = -g -Wall -Werror -std=c++0x # -std=c++11
CFLAGS = -I./include
LIBS = -lboost_unit_test_framework

.PHONY: all
all: test

test: src/test.cpp $(shell find include -type f -name "*.hpp")
	$(CXX) $(CXXFLAGS) $(CFLAGS) $(LIBS) -o $@ $< $(LIBS)

.PHONY: check
check: test
	./test

clean:
	$(RM) test

