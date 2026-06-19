# mini_ode_robots — Makefile
#
# Build rules follow the ODE skill's hard rules:
#  - link with the C++ driver (c++) using ode-config for flags  (skill rule #7)
#  - define NO precision macro; <ode/ode.h> already matches the lib
#  - headless: no rendering dependency at all
#
# Targets:
#   make            -> build the static lib + all examples
#   make test       -> build & run the math + physics self-checks (headless)
#   make clean

CXX      ?= c++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations
ODE_CFLAGS := $(shell ode-config --cflags)
ODE_LIBS   := $(shell ode-config --libs)
INC        := -Iinclude -Irobots

SRC   := $(wildcard src/*.cpp) $(wildcard robots/*.cpp)
OBJ   := $(patsubst %.cpp,build/%.o,$(notdir $(SRC)))
LIB   := build/libmor.a

EXAMPLES := test_math selfcheck test_features demo_robot demo_arm demo_snake demo_sensors
BINS     := $(addprefix build/,$(EXAMPLES))

VPATH := src:robots:examples

.PHONY: all test clean
all: $(LIB) $(BINS)

$(LIB): $(OBJ)
	ar rcs $@ $^

build/%.o: %.cpp | build
	$(CXX) $(CXXFLAGS) $(ODE_CFLAGS) $(INC) -c $< -o $@

build/%: examples/%.cpp $(LIB) | build
	$(CXX) $(CXXFLAGS) $(ODE_CFLAGS) $(INC) $< $(LIB) $(ODE_LIBS) -o $@

build:
	@mkdir -p build

test: all
	@echo "==== math convention test ===="   && ./build/test_math
	@echo "==== physics self-check ===="      && ./build/selfcheck
	@echo "==== feature tests ===="           && ./build/test_features
	@echo "==== robot trajectory demo ===="   && ./build/demo_robot
	@echo "==== servo arm demo ===="          && ./build/demo_arm
	@echo "==== anisotropic snake demo ===="  && ./build/demo_snake
	@echo "==== composable sensors demo ===="  && ./build/demo_sensors

clean:
	rm -rf build
