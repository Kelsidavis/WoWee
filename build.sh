#!/bin/bash
# Wowee Build Script - Ensures no stale binaries

set -e  # Exit on error

cd "$(dirname "$0")"

echo "Building wowee..."

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure with cmake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build with all cores
echo "Building with $(nproc) cores..."
cmake --build . --parallel $(nproc)

# Ensure Data symlink exists in bin directory
cd bin
if [ ! -e Data ]; then
    ln -s ../../Data Data
fi
cd ..

echo ""
echo "Build complete! Binary: build/bin/wowee"
echo "Run with: cd build/bin && ./wowee"
