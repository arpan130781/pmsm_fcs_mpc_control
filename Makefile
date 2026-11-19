PYTHON ?= $(shell if [ -f ".venv/bin/python3" ]; then echo ".venv/bin/python3"; else which python3; fi)
MATLAB ?= $(shell if [ -d "/Applications/MATLAB_R2024a.app/bin" ]; then echo "/Applications/MATLAB_R2024a.app/bin/matlab"; elif [ -d "/Applications/MATLAB_R2022b.app/bin" ]; then echo "/Applications/MATLAB_R2022b.app/bin/matlab"; else which matlab; fi)

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

.PHONY: all lib test pytest test-all clean run simulink simulink-sim simulink-test

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

simulink-build:
	@echo "=== Compiling MEX S-Functions for Simulink ==="
	cd simulink && $(MATLAB) -batch "compile_all_sfunctions; create_pmsm_mpc_model"

simulink-sim:
	@echo "=== Executing FCS-MPC Simulink Co-Simulation ==="
	cd simulink && $(MATLAB) -batch "run_simulink_mpc_simulation"

simulink-test:
	@echo "=== Running FCS-MPC Automated Simulink Verification Suite ==="
	cd simulink && $(MATLAB) -batch "test_simulink_suite"

simulink: simulink-build simulink-sim
