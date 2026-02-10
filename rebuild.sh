#!/bin/bash
# Wowee Clean Rebuild Script - Removes all build artifacts and rebuilds from scratch

set -e  # Exit on error

cd "$(dirname "$0")"

echo "Clean rebuilding wowee..."

# Remove build directory completely
if [ -d "build" ]; then
    echo "Removing old build directory..."
    rm -rf build
fi

# Create fresh build directory
mkdir -p build
cd build

# Configure with cmake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build with all cores
echo "Building with $(nproc) cores..."
cmake --build . --parallel $(nproc)

# Create Data symlink in bin directory
echo "Creating Data symlink..."
cd bin
if [ ! -e Data ]; then
    ln -s ../../Data Data
    echo "  Created Data -> ../../Data"
fi
cd ..

echo ""
echo "Clean build complete! Binary: build/bin/wowee"
echo "Run with: cd build/bin && ./wowee"
