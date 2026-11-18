#!/bin/sh
# Standalone math-library build. No ROS, sensors, control, or flight actions.
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
package_dir="$project_dir/src/airseafly_real400_mission"
output_dir="$project_dir/runtime/fixed_trajectory"
eigen_include=${EIGEN3_INCLUDE_DIR:-}
if [ -z "$eigen_include" ]; then
  for candidate in /usr/include/eigen3 /opt/homebrew/include/eigen3 /usr/local/include/eigen3; do
    if [ -f "$candidate/Eigen/Core" ]; then eigen_include=$candidate; break; fi
  done
fi
if [ -z "$eigen_include" ] || [ ! -f "$eigen_include/Eigen/Core" ]; then
  echo 'Eigen3 headers not found. Install libeigen3-dev (Ubuntu) or eigen (Homebrew), or set EIGEN3_INCLUDE_DIR.' >&2
  exit 2
fi
mkdir -p "$output_dir"
case $(uname -s) in
  Darwin) suffix=dylib; link_flag=-dynamiclib ;;
  *) suffix=so; link_flag=-shared ;;
esac
"${CXX:-c++}" -std=c++14 -O2 -fPIC "$link_flag" \
  -I "$eigen_include" -I "$package_dir/vendor/gcopter/include" \
  "$package_dir/native/fixed_min_snap.cpp" \
  -o "$output_dir/libairseafly_fixed_min_snap.$suffix"
printf '%s\n' "$output_dir/libairseafly_fixed_min_snap.$suffix"
