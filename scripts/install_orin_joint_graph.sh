#!/usr/bin/env bash

set -euo pipefail

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: install_orin_joint_graph.sh [--workspace DIR] [--force]

Installs four motor-policy bundles into DIR/models:
  v12b_go2_motor, v12c_go2_motor, v12b_b2w_motor, v12c_b2w_motor

Existing generated bundles are preserved unless --force is passed. With
--force, each old bundle is moved to a timestamped backup before replacement.
EOF
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_dir="$(cd -- "${script_dir}/.." && pwd -P)"
if [[ "$(basename -- "$(dirname -- "${repo_dir}")")" == "src" ]]; then
  default_workspace="$(cd -- "${repo_dir}/../.." && pwd -P)"
else
  default_workspace="${STEPIT_WS:-}"
fi

workspace_dir="${STEPIT_WS:-${default_workspace}}"
force=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      [[ $# -ge 2 ]] || die "--workspace requires a directory"
      workspace_dir="$2"
      shift 2
      ;;
    --force)
      force=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ -n "${workspace_dir}" ]] || die "set STEPIT_WS or pass --workspace"
workspace_dir="$(cd -- "${workspace_dir}" && pwd -P)"
[[ -f "${workspace_dir}/src/stepit/CMakeLists.txt" ]] \
  || die "expected ${workspace_dir}/src/stepit/CMakeLists.txt"
[[ "$(cd -- "${workspace_dir}/src/stepit" && pwd -P)" == "${repo_dir}" ]] \
  || die "this installer must run from the Stepit checkout at ${workspace_dir}/src/stepit"

python3 "${repo_dir}/scripts/verify_orin_joint_graph.py" --repo "${repo_dir}"

models_dir="${workspace_dir}/models"
mkdir -p "${models_dir}"
staging_dir="$(mktemp -d "${models_dir}/.orin_joint_graph.XXXXXX")"
trap 'rm -rf -- "${staging_dir}"' EXIT
timestamp="$(date +%Y%m%d_%H%M%S)"

install_bundle() {
  local variant="$1"
  local profile="$2"
  local policy_name="mirlab_joint_graph_${variant}_${profile}"
  local source_actor="${repo_dir}/deploy/orin/actors/${variant}"
  local source_profile="${repo_dir}/deploy/orin/profiles/${profile}"
  local staged="${staging_dir}/${policy_name}"
  local destination="${models_dir}/${policy_name}"

  mkdir -p "${staged}"
  install -m 0644 "${source_actor}/actor.yml" "${staged}/actor.yml"
  install -m 0644 "${source_actor}/manifest.json" "${staged}/actor_manifest.json"
  install -m 0644 "${source_profile}/profile.json" "${staged}/profile.json"
  install -m 0644 "${source_profile}/joint_graph_observation.yml" "${staged}/joint_graph_observation.yml"
  install -m 0644 "${source_profile}/joint_graph_motor_action.yml" "${staged}/joint_graph_motor_action.yml"
  install -m 0644 "${source_profile}/joint_graph_action_reordering.yml" "${staged}/joint_graph_action_reordering.yml"
  install -m 0644 "${source_profile}/joint_graph_path_subscriber.yml" "${staged}/joint_graph_path_subscriber.yml"
  sed -e "s/@POLICY_NAME@/${policy_name}/g" -e "s/@VARIANT@/${variant}/g" \
    "${source_profile}/policy.yml.in" >"${staged}/policy.yml"
  ln -s "../../src/stepit/deploy/orin/actors/${variant}/actor.onnx" "${staged}/actor.onnx"

  if [[ -e "${destination}" || -L "${destination}" ]]; then
    [[ "${force}" == true ]] || die "${destination} exists; pass --force to preserve it as a backup"
    mv -- "${destination}" "${destination}.backup_${timestamp}"
  fi
  mv -- "${staged}" "${destination}"
  printf 'Installed %s\n' "${destination}"
}

for variant in v12b v12c; do
  install_bundle "${variant}" go2_motor
  install_bundle "${variant}" b2w_motor
done

python3 "${repo_dir}/scripts/verify_orin_joint_graph.py" \
  --repo "${repo_dir}" --workspace "${workspace_dir}"
printf 'Orin joint-graph bundles are ready under %s\n' "${models_dir}"
