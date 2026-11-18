#!/usr/bin/env bash
# Offline algorithm tests. NO ROS master, controller, network or flight command.
# Usage: bash run_tests.sh /path/to/eigen3
set -euo pipefail
test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd "${test_dir}/../../../../.." && pwd)"
eigen_include="${1:-/usr/include/eigen3}"
test_build="$(mktemp -d "${TMPDIR:-/tmp}/airseafly-3d_ceiling-tests.XXXXXX")"
compiler="${CXX:-c++}"
compiler_flags=(-std=c++14 -O1 -g -Wno-narrowing)
if [[ "${CEILING_SANITIZE:-0}" == "1" ]]; then
  compiler_flags+=("-fsanitize=${CEILING_SANITIZERS:-address,undefined}" -fno-omit-frame-pointer)
fi
for algorithm_test in test_3d_ceiling test_short_planner; do
"${compiler}" "${compiler_flags[@]}" \
  -I"${eigen_include}" -I"${test_dir}/stubs" \
  -I"${project_dir}/src/planner/bspline_opt/include" \
  -I"${project_dir}/src/planner/plan_env/include" \
  -I"${project_dir}/src/planner/path_searching/include" \
  -I"${project_dir}/src/planner/plan_manage/include" \
  -I"${project_dir}/src/planner/traj_utils/include" \
  "${test_dir}/${algorithm_test}.cpp" \
  "${project_dir}/src/planner/bspline_opt/src/uniform_bspline.cpp" \
  "${project_dir}/src/planner/bspline_opt/src/bspline_optimizer.cpp" \
  "${project_dir}/src/planner/path_searching/src/dyn_a_star.cpp" \
  "${project_dir}/src/planner/plan_manage/src/planner_manager.cpp" \
  "${project_dir}/src/planner/traj_utils/src/polynomial_traj.cpp" \
  -o "${test_build}/${algorithm_test}"
"${test_build}/${algorithm_test}"
done
"${compiler}" "${compiler_flags[@]}" \
  -I"${eigen_include}" -I"${test_dir}/stubs" \
  -I"${project_dir}/src/planner/bspline_opt/include" \
  -I"${project_dir}/src/planner/plan_env/include" \
  -I"${project_dir}/src/planner/path_searching/include" \
  -I"${project_dir}/src/planner/traj_utils/include" \
  "${test_dir}/test_collision_sampling.cpp" \
  "${project_dir}/src/planner/bspline_opt/src/uniform_bspline.cpp" \
  "${project_dir}/src/planner/bspline_opt/src/bspline_optimizer.cpp" \
  "${project_dir}/src/planner/path_searching/src/dyn_a_star.cpp" \
  -o "${test_build}/test_collision_sampling"
"${test_build}/test_collision_sampling"
printf 'Offline test executable: %s\n' "${test_build}/test_3d_ceiling"
