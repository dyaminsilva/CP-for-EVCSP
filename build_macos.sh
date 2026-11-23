#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Path to the IBM ILOG CPLEX Optimization Studio installation.
studio="${CPLEX_STUDIO_DIR:-/Applications/CPLEX_Studio2211}"

clang++ -std=c++20 -O2 -DNDEBUG -DIL_STD -arch arm64 -stdlib=libc++ -fPIC -fexceptions -fno-strict-aliasing \
    -Icpp \
    -I"$studio/cplex/include" \
    -I"$studio/cpoptimizer/include" \
    -I"$studio/concert/include" \
    cpp/main.cpp \
    cpp/model/EVCSP.cpp \
    cpp/model/ChargingCommon.cpp \
    cpp/model/CP.cpp \
    cpp/model/CostPropagation.cpp \
    cpp/model/MIP.cpp \
    cpp/model/ChargingNetwork.cpp \
    cpp/model/CostLP.cpp \
    -L"$studio/cplex/lib/arm64_osx/static_pic" \
    -L"$studio/cpoptimizer/lib/arm64_osx/static_pic" \
    -L"$studio/concert/lib/arm64_osx/static_pic" \
    -lilocplex -lcp -lconcert -lcplex -lm -lpthread -framework CoreFoundation -framework IOKit \
    -o CP-for-EVCSP

echo "Built $PWD/CP-for-EVCSP"
