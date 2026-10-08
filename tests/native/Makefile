FW      := ../../firmware/BikeSafetyBelt
SRCS    := $(shell find $(FW)/src -name '*.cpp') native_platform.cpp test_main.cpp
CXX     ?= g++
CXXFLAGS := -std=c++11 -O2 -Wall -Wextra -Wno-unused-parameter -I$(FW) -I.

all: run

bsb_tests: $(SRCS) $(shell find $(FW) -name '*.h')
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ -lm

run: bsb_tests
	./bsb_tests ../../app/samples/demo-session.log

clean:
	rm -f bsb_tests

.PHONY: all run clean
