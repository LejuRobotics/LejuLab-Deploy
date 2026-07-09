#!/bin/bash
# Generate Protobuf C++ files for leju-remote
# Usage: ./generate.sh
# Note: Requires protoc 3.15+ for optional keyword support.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
PROTOS_DIR="$PKG_DIR/protos"

echo "=== Protobuf C++ Generator (leju-remote) ==="
echo "Protoc version: $(protoc --version)"
echo "Proto files dir: $PROTOS_DIR"
echo "Output dir: $SCRIPT_DIR"
echo ""

# Check proto files exist
for f in "$PROTOS_DIR/protos/hand_pose.proto" "$PROTOS_DIR/protos/hand_wrench_srv.proto" "$PROTOS_DIR/protos/robot_info.proto"; do
  if [ ! -f "$f" ]; then
    echo "Error: Proto file not found: $f"
    exit 1
  fi
done

# Generate C++ files (proto_path is parent of protos/ so that "protos/hand_pose.proto" works)
echo "Generating C++ files..."
protoc --proto_path="$PROTOS_DIR" --cpp_out="$SCRIPT_DIR" \
  protos/hand_pose.proto \
  protos/hand_wrench_srv.proto \
  protos/robot_info.proto

# Fix include paths (protos/hand_wrench_srv.pb.h -> hand_wrench_srv.pb.h)
echo "Fixing include paths..."
cd "$SCRIPT_DIR"
for file in protos/*.pb.cc protos/*.pb.h; do
  [ -f "$file" ] || continue
  sed -i 's|#include "protos/\([^"]*\)"|#include "\1"|g' "$file"
done

# Move files from protos/ to protos_c/ if generated in subdir
if [ -d "protos" ]; then
  mv protos/*.pb.cc protos/*.pb.h . 2>/dev/null || true
  rmdir protos 2>/dev/null || true
fi

echo ""
echo "=== Generated files ==="
ls -la *.pb.cc *.pb.h 2>/dev/null || echo "No files generated"
echo ""
echo "Done!"
