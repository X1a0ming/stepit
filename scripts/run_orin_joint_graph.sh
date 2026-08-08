#!/usr/bin/env bash

set -euo pipefail

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: run_orin_joint_graph.sh MODEL MODE [options]

MODEL: v12b | v12c
MODE:  go2_motor | b2w_motor | cmd_vel

Options:
  --workspace DIR       Stepit workspace (default: STEPIT_WS)
  --netif NAME          Unitree robot network interface for motor modes
  --unitree-domain ID   Unitree DDS domain (default: 0)
  --ros-domain ID       ROS 2 domain (default: current ROS_DOMAIN_ID or 0)

Motor modes start Stepit in Resting state. Use Agent/StandUp and then
Agent/PolicyOn only after completing the hardware mapping checks.
EOF
}

[[ $# -ge 2 ]] || { usage; exit 2; }
variant="$1"
mode="$2"
shift 2

case "${variant}" in v12b|v12c) ;; *) die "MODEL must be v12b or v12c" ;; esac
case "${mode}" in go2_motor|b2w_motor|cmd_vel) ;; *) die "unsupported MODE: ${mode}" ;; esac

workspace_dir="${STEPIT_WS:-}"
netif="${STEPIT_NETIF:-}"
unitree_domain="${STEPIT_UNITREE2_DOMAIN_ID:-0}"
ros_domain="${ROS_DOMAIN_ID:-0}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) [[ $# -ge 2 ]] || die "--workspace requires a value"; workspace_dir="$2"; shift 2 ;;
    --netif) [[ $# -ge 2 ]] || die "--netif requires a value"; netif="$2"; shift 2 ;;
    --unitree-domain) [[ $# -ge 2 ]] || die "--unitree-domain requires a value"; unitree_domain="$2"; shift 2 ;;
    --ros-domain) [[ $# -ge 2 ]] || die "--ros-domain requires a value"; ros_domain="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ -n "${workspace_dir}" ]] || die "set STEPIT_WS or pass --workspace"
workspace_dir="$(cd -- "${workspace_dir}" && pwd -P)"
repo_dir="${workspace_dir}/src/stepit"
[[ -f "${workspace_dir}/install/setup.bash" ]] || die "build the ROS 2 workspace first"

set +u
source "${workspace_dir}/install/setup.bash"
set -u
export STEPIT_WS="${workspace_dir}"
export ROS_DOMAIN_ID="${ros_domain}"

if [[ "${mode}" == "cmd_vel" ]]; then
  actor_dir="${repo_dir}/deploy/orin/actors/${variant}"
  python3 "${repo_dir}/scripts/verify_orin_joint_graph.py" --repo "${repo_dir}"
  exec ros2 run stepit_ros2 joint_graph_cmd_vel_node --ros-args \
    --params-file "${repo_dir}/deploy/orin/cmd_vel/params.yml" \
    -p "model_path:=${actor_dir}/actor.onnx" \
    -p "model_config_path:=${actor_dir}/actor.yml"
fi

[[ -n "${netif}" ]] || die "motor mode requires --netif or STEPIT_NETIF"
[[ "${netif}" != "lo" ]] || die "refusing loopback for a real motor deployment"
ip link show "${netif}" >/dev/null 2>&1 || die "network interface ${netif} does not exist"
export STEPIT_NETIF="${netif}"
export STEPIT_UNITREE2_DOMAIN_ID="${unitree_domain}"
export LD_LIBRARY_PATH="${repo_dir}/extern/robot_sdk/unitree_sdk2/thirdparty/lib/$(uname -m)${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

bundle="${workspace_dir}/models/mirlab_joint_graph_${variant}_${mode}"
[[ -f "${bundle}/policy.yml" ]] || die "missing ${bundle}; run install_orin_joint_graph.sh first"
python3 "${repo_dir}/scripts/verify_orin_joint_graph.py" \
  --repo "${repo_dir}" --workspace "${workspace_dir}"

robot="${mode%_motor}"
exec "${repo_dir}/scripts/run.sh" --workspace "${workspace_dir}" --build-tool colcon \
  -c console -c ros2_srv -P ros2 -r "${robot}" -p "neuro@${bundle}" \
  -- --ros-args --remap __ns:=/stepit
