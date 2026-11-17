PYTHON ?= python3

CC=gcc
CFLAGS=-Wall -Wextra -O3 -fPIC
SRC_DIR=src
BUILD_DIR=build

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
	LIB_EXT := dylib
else
	LIB_EXT := so
endif

LIB_TARGET := $(BUILD_DIR)/libmpc_controller.$(LIB_EXT)
LEGACY_LIB := $(BUILD_DIR)/libfoc_controller.$(LIB_EXT)
SRCS := $(filter-out $(SRC_DIR)/tests/%, $(wildcard $(SRC_DIR)/*.c))

.PHONY: all lib test pytest test-all clean run

all: lib test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

lib: $(BUILD_DIR)
	$(CC) $(CFLAGS) -shared $(SRCS) -lm -o $(LIB_TARGET)
	cp $(LIB_TARGET) $(LEGACY_LIB)

test: lib
	$(CC) $(CFLAGS) $(SRC_DIR)/tests/test_mpc.c $(SRCS) -lm -o $(BUILD_DIR)/test_mpc
	./$(BUILD_DIR)/test_mpc

pytest:
	$(PYTHON) -m pytest simulation/ -v

test-all: test pytest

clean:
	rm -rf $(BUILD_DIR)

run: lib
	$(PYTHON) simulation/harness/mpc_runner.py
