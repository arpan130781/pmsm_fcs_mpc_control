#!/bin/bash
set -e

echo "=================================================================="
echo "Running FCS-MPC Power Electronics Drive Benchmark Suite"
echo "Author: Arpan Kailash Gupta (241030009), VJTI Mumbai"
echo "=================================================================="

# 1. Compile C library and unit tests
make all

# 2. Run Python FCS-MPC Simulation Benchmarks
${PYTHON:-python3} simulation/harness/mpc_runner.py

echo ""
echo "[COMPLETE] All FCS-MPC benchmarks and unit tests executed successfully!"
