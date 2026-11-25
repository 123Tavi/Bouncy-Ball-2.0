CXX      := clang++
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -Wpedantic -Iinclude

BREW_PREFIX := /opt/homebrew
SFML_INC    := -I$(BREW_PREFIX)/include
SFML_LIB    := -L$(BREW_PREFIX)/lib -Wl,-rpath,$(BREW_PREFIX)/lib \
               -lsfml-graphics -lsfml-window -lsfml-audio -lsfml-system

TARGET      := bouncy_ball
TEST_TARGET := build/physics_tests

.PHONY: all run test clean

all: $(TARGET)

build:
	mkdir -p build

build/Physics.o: src/Physics.cpp include/Physics.hpp include/Vec2.hpp | build
	$(CXX) $(CXXFLAGS) -c src/Physics.cpp -o $@

build/main.o: src/main.cpp include/Physics.hpp include/Vec2.hpp include/AudioSynth.hpp | build
	$(CXX) $(CXXFLAGS) $(SFML_INC) -c src/main.cpp -o $@

$(TARGET): build/Physics.o build/main.o
	$(CXX) $(CXXFLAGS) $^ -o $@ $(SFML_LIB)

$(TEST_TARGET): build/Physics.o tests/physics_tests.cpp | build
	$(CXX) $(CXXFLAGS) build/Physics.o tests/physics_tests.cpp -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf build $(TARGET)
