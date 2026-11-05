#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# IBM CPLEX Studio installation on servers 4 and 5.
studio=/opt/ibm/ILOG/CPLEX_Studio221

g++ -std=c++20 -O2 -DNDEBUG -DIL_STD -m64 -fPIC -fno-strict-aliasing \
    -Icpp \
    -I"$studio/cplex/include" \
    -I"$studio/cpoptimizer/include" \
    -I"$studio/concert/include" \
    cpp/main.cpp \
    cpp/model/EVCSP.cpp \
    cpp/model/ChargingCommon.cpp \
    cpp/model/ChargingCP.cpp \
    cpp/model/ChargingMILP.cpp \
    -L"$studio/cplex/lib/x86-64_linux/static_pic" \
    -L"$studio/cpoptimizer/lib/x86-64_linux/static_pic" \
    -L"$studio/concert/lib/x86-64_linux/static_pic" \
    -lilocplex -lcp -lconcert -lcplex -lm -lpthread -ldl \
    -o CP-for-EVCSP

echo "Built $PWD/CP-for-EVCSP"
