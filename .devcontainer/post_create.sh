#!/bin/bash

set -e

echo "Running post-create setup..."

# Update rosdep
rosdep update

# Install dependencies from workspace
cd /workspace
rosdep install --from-paths src --ignore-src -r -y || true

echo "Post-create setup complete!"
