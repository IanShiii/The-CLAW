#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script location: $SCRIPT_DIR"

rm -rf "$SCRIPT_DIR/assets"
rm -rf "$SCRIPT_DIR/robot.urdf"

onshape-to-robot "$SCRIPT_DIR/onshape_to_robot_config.json"

sed -i 's|package://|package://robot_description/urdf/|g' "$SCRIPT_DIR/robot.urdf"