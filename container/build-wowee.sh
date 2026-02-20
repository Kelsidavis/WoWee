#!/bin/bash

set -eu 
set -o pipefail

cp -r /WoWee-src /WoWee

pushd /WoWee
./build.sh
popd

pushd /WoWee/build
cmake --install . --prefix=/build
popd
