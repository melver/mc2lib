#
# Test Makefile for mc2lib
#

CXX = g++
CXXFLAGS = -g -Wall -Werror -std=c++0x # -std=c++11
CFLAGS = -I./include
LIBS = -lboost_unit_test_framework
DOXYGEN = doxygen

.PHONY: all
all: test

# Need to append $(LIBS) again for gcc 4.6
test: src/test.cpp $(shell find include -type f -name "*.hpp")
	$(CXX) $(CXXFLAGS) $(CFLAGS) $(LIBS) -o $@ $< $(LIBS)

.PHONY: tidy
tidy: compile_commands.json
	clang-tidy \
		-header-filter='.*' \
		-checks='-*,clang-analyzer-*,google*,misc*' \
		src/test.cpp

compile_commands.json: Makefile
	echo '[ { "directory" : "$(PWD)",' \
		'"command" : "/usr/bin/clang++ $(CXXFLAGS) $(CFLAGS) $(LIBS) -o test src/test.cpp",' \
		'"file": "src/test.cpp" } ]' > compile_commands.json

.PHONY: check
check: test
	./test

.PHONY: doc
doc:
	$(RM) -r apidoc/html
	$(DOXYGEN) Doxyfile

clean:
	$(RM) test

