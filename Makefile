#
# Test Makefile for mc2lib
#

CXX ?= g++
DOXYGEN ?= doxygen
CLANG_TIDY ?= clang-tidy
CLANG_FORMAT ?= clang-format
CPPLINT ?= cpplint

GTEST_DIR = third_party/googletest/googletest
GMOCK_DIR = third_party/googletest/googlemock

BUILDFLAGS = -g
WARNFLAGS = -Wall -Werror
INCLUDES = -isystem $(GTEST_DIR)/include -isystem $(GMOCK_DIR)/include -I./include
CXXFLAGS = -std=c++11 -pthread
LIBS =

gtestmock_MODULES = build/$(GTEST_DIR)/src/gtest-all.cc.o build/$(GMOCK_DIR)/src/gmock-all.cc.o

HEADER_FILES = $(shell find include -name "*.hpp")

test_mc2lib_MODULES = $(shell find src -name "test_*.cpp" | while read line; do echo "build/$${line}.o"; done)

.PHONY: all
all: test_mc2lib

test_mc2lib: $(gtestmock_MODULES) $(test_mc2lib_MODULES)
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(WARNFLAGS) $(LIBS) $^ -o $@

build/src/%.cpp.o: src/%.cpp $(HEADER_FILES)
	@mkdir -pv $$(dirname $@)
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(WARNFLAGS) $(INCLUDES) -c -o $@ $<

build/third_party/googletest/%.cc.o: third_party/googletest/%.cc
	@mkdir -pv $$(dirname $@)
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -I$(GTEST_DIR) -I$(GMOCK_DIR) $(INCLUDES) -c -o $@ $^

.PHONY: tidy
tidy: compile_commands.json
	$(CLANG_TIDY) \
		-header-filter='.*' \
		-checks='-*,clang-analyzer-*,google*,misc*,-misc-unused-parameters,performance*' \
		$(shell find src -name "*.cpp")

compile_commands.json: Makefile
	echo "[" > compile_commands.json
	for obj in $(test_mc2lib_MODULES); do \
		src=$${obj%.o}; src=$${src#build/}; \
		echo "$${delim}{ \"directory\" : \"$(PWD)\"," \
			"\"command\" : \"/usr/bin/clang++ $(CXXFLAGS) $(BUILDFLAGS) $(WARNFLAGS) $(INCLUDES) -c -o $${obj} $${src}\"," \
			"\"file\": \"$${src}\" }" >> compile_commands.json; \
		delim=","; \
	done
	echo "]" >> compile_commands.json

.PHONY: format
format:
	@echo "Modifying files in-place..."
	$(CLANG_FORMAT) \
		-style=file \
		-i \
		$(shell git ls-files | grep -E '\.(hpp|cpp)')

.PHONY: lint
lint:
	$(CPPLINT) \
		--extensions=cpp,hpp \
		--filter=-build/c++11 \
		$(shell git ls-files | grep -E '\.(hpp|cpp)')

.PHONY: check
check: test_mc2lib
	./test_mc2lib

.PHONY: doc
doc:
	$(RM) -r apidoc/html
	$(DOXYGEN) Doxyfile

.PHONY: clean
clean:
	$(RM) $(test_mc2lib_MODULES)
	$(RM) test_mc2lib

.PHONY: cleanall
cleanall: clean
	$(RM) compile_commands.json
	$(RM) -r build
