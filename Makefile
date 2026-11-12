## The build now uses CMake (needed for C++17, the nlohmann::json dependency
## and the Catch2 test suite). This Makefile is kept only as a convenience
## wrapper around the CMake commands.

default: build

build:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
	cmake --build build -j

test: build
	ctest --test-dir build --output-on-failure

clean:
	rm -rf build snapshot.json

.PHONY: default build test clean
