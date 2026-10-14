#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "$0")/.." && pwd)"
git clone --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git "$task_root/vendor/unitree_ros"
git -C "$task_root/vendor/unitree_ros" sparse-checkout set robots/g1_description
git -C "$task_root/vendor/unitree_ros" checkout 5994d4faef0a9cadd3287f8de0199a67eeb2a259
