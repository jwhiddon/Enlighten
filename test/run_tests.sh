#!/bin/sh
# Build and run the host-side test suite (requires g++ on PATH).
set -e
root="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$root/build"
g++ -std=c++14 -O2 -Wall -Wextra -Werror \
    -I "$root/../Enlighten/src" -I "$root" \
    "$root"/../Enlighten/src/core/*.cpp "$root"/*.cpp \
    -o "$root/build/tests"
exec "$root/build/tests"
