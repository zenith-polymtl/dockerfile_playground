#!/usr/bin/env bash
set -euo pipefail

# Repo URLs and where to clone them
declare -A repos=(
  [ros2_ws/src]="https://github.com/zenith-polymtl/ros2-mission-2"
)

for target in "${!repos[@]}"; do
  url=${repos[$target]}

  if [ -d "$target" ]; then
    echo "✔ $target already exists; switching to dev"
    git -C "$target" fetch origin
    git -C "$target" checkout dev
    git -C "$target" pull --ff-only
  else
    echo "⏳ Cloning $url → $target (dev branch)"
    mkdir -p "$(dirname "$target")"
    git clone --branch dev "$url" "$target"
  fi
done


docker compose up --build