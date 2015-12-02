#
# Test Makefile for mc2lib
#

CXX = g++
CXXFLAGS = -g -Wall -Werror -std=c++0x # -std=c++11
CFLAGS = -I./include
LIBS = -lboost_unit_test_framework
DOXYGEN = doxygen
CLANG_TIDY = clang-tidy
CLANG_FORMAT = clang-format
CPPLINT = cpplint

.PHONY: all
all: test_mc2lib

TEST_SRC = $(shell find src -name "test_*.cpp")

# Need to append $(LIBS) again for gcc 4.6
test_mc2lib: $(TEST_SRC) $(shell find include -type f -name "*.hpp")
	$(CXX) $(CXXFLAGS) $(CFLAGS) $(LIBS) -o $@ $(TEST_SRC) $(LIBS)

.PHONY: tidy
tidy: compile_commands.json
	$(CLANG_TIDY) \
		-header-filter='.*' \
		-checks='-*,clang-analyzer-*,google*,misc*' \
		$(TEST_SRC)

compile_commands.json: Makefile
	echo "[" > compile_commands.json
	for s in $(TEST_SRC); do \
		echo "$${d}{ \"directory\" : \"$(PWD)\"," \
			"\"command\" : \"/usr/bin/clang++ $(CXXFLAGS) $(CFLAGS) $(LIBS) -c -o $${s}.o $${s}\"," \
			"\"file\": \"$${s}\" }" >> compile_commands.json; \
		d=","; \
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
	./test_mc2lib --report_level=detailed

.PHONY: doc
doc:
	$(RM) -r apidoc/html
	$(DOXYGEN) Doxyfile

clean:
	$(RM) test_mc2lib

