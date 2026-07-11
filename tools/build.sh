#!/bin/sh
# Build the sequence generator and show player (requires g++ on PATH).
set -e
root="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$root/build"
inc="-I $root/../Enlighten/src -I $root/../test"
g++ -std=c++14 -O2 -Wall -Wextra -Werror $inc \
    "$root"/../Enlighten/src/core/*.cpp "$root/../test/sim.cpp" \
    "$root/seqgen.cpp" -o "$root/build/seqgen"
g++ -std=c++14 -O2 -Wall -Wextra -Werror $inc \
    "$root"/../Enlighten/src/core/*.cpp "$root/../test/sim.cpp" \
    "$root/showplay.cpp" -o "$root/build/showplay"
g++ -std=c++14 -O2 -Wall -Wextra -Werror $inc \
    "$root"/../Enlighten/src/core/*.cpp "$root/../test/sim.cpp" \
    "$root/showviz.cpp" -o "$root/build/showviz"
echo "built: $root/build/seqgen, $root/build/showplay, $root/build/showviz"
