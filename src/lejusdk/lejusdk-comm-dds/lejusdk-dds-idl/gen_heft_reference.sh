#!/bin/bash
# Generate C++ for HeftReference.idl using the repo's bundled idlc (x86_64 host).
# Mirrors gen_idl_cxx.sh but processes a single file and works without idlc on PATH.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ARCH=$(uname -m)
THIRD_PARTY="$SCRIPT_DIR/../../3rd_party/$ARCH"
export PATH="$THIRD_PARTY/cyclonedds-0.10.2/bin:$PATH"
export LD_LIBRARY_PATH="$THIRD_PARTY/cyclonedds-0.10.2/lib:$THIRD_PARTY/cyclonedds-cxx-0.10.2/lib:$LD_LIBRARY_PATH"

mkdir -p include/lejusdk-dds-idl src-generated

file="idl/HeftReference.idl"
base=$(basename "$file" .idl)
echo "Processing $file"
idlc -l cxx -o . "$file"
mv "$base.hpp" include/lejusdk-dds-idl/
mv "$base.cpp" src-generated/
echo "Generated include/lejusdk-dds-idl/$base.hpp and src-generated/$base.cpp"
