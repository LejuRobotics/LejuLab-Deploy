#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

mkdir -p include src-generated

echo "Generating C++ files from IDL..."

mkdir -p include/lejusdk-dds-idl

# Process all IDL files including subdirectories
idl_files=$(find idl -name "*.idl" 2>/dev/null)
if [ -z "$idl_files" ]; then
    echo "No IDL files found in idl/"
    exit 1
fi

for file in $idl_files; do
    echo "Processing $file"
    base=$(basename "$file" .idl)
    if idlc -l cxx -o . "$file"; then
        if [ -f "$base.hpp" ]; then
            mv "$base.hpp" include/lejusdk-dds-idl/
            echo "  ✓ Generated $base.hpp"
        else
            echo "  ✗ Failed to find $base.hpp"
            exit 1
        fi

        if [ -f "$base.cpp" ]; then
            mv "$base.cpp" src-generated/
            echo "  ✓ Generated $base.cpp"
        else
            echo "  ✗ Failed to find $base.cpp"
            exit 1
        fi
    else
        echo "  ✗ Failed to process $file"
        exit 1
    fi
done

echo ""
echo "Generated files:"
echo "Headers:"
ls -la include/lejusdk-dds-idl/*.hpp 2>/dev/null || echo "  No .hpp files found"
echo ""
echo "Sources:"
ls -la src-generated/*.cpp 2>/dev/null || echo "  No .cpp files found"