#!/bin/sh
# Build and run the feature tour (requires g++ on PATH).
set -e
root="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$root/build"
g++ -std=c++14 -O2 -Wall -Wextra -Werror \
    -I "$root/../Enlighten/src" -I "$root/../test" \
    "$root"/../Enlighten/src/core/*.cpp "$root/../test/sim.cpp" \
    "$root/feature_tour.cpp" \
    -o "$root/build/feature_tour"
exec "$root/build/feature_tour"
