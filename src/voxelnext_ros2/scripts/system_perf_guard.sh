#!/usr/bin/env bash
set -euo pipefail

# System-side guard for stabilizing ROS2 perception latency on laptops.
# - status: show current thermal/power state
# - apply:  apply conservative CPU limits (requires root)
# - revert: restore settings captured by apply (requires root)
# - watch:  live telemetry for diagnosing throttle/jitter

# Original baseline captured once at first apply in a cycle.
ORIGINAL_BACKUP_FILE="/tmp/voxelnext_sys_perf_guard_original.env"
# Snapshot of state right before the latest apply (for diagnostics only).
LAST_APPLY_SNAPSHOT_FILE="/tmp/voxelnext_sys_perf_guard_last_apply.env"
INTEL_PSTATE_DIR="/sys/devices/system/cpu/intel_pstate"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

need_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    err "root 권한이 필요합니다. sudo로 다시 실행하세요."
    exit 1
  fi
}

sum_throttle_counts() {
  local pattern="$1"
  local sum=0
  local f
  shopt -s nullglob
  for f in ${pattern}; do
    if [[ -f "${f}" ]]; then
      sum=$((sum + $(<"${f}")))
    fi
  done
  shopt -u nullglob
  echo "${sum}"
}

list_epp_files() {
  local files=()
  shopt -s nullglob
  files=(/sys/devices/system/cpu/cpufreq/policy*/energy_performance_preference)
  if (( ${#files[@]} == 0 )); then
    files=(/sys/devices/system/cpu/cpu[0-9]*/cpufreq/energy_performance_preference)
  fi
  printf '%s\n' "${files[@]}"
  shopt -u nullglob
}

print_cpu_policy_summary() {
  local govs epps
  govs="$(for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do [[ -f "${g}" ]] && cat "${g}"; done | sort | uniq -c | xargs || true)"
  epps="$(while IFS= read -r e; do [[ -f "${e}" ]] && cat "${e}"; done < <(list_epp_files) | sort | uniq -c | xargs || true)"
  info "governor summary: ${govs:-N/A}"
  info "EPP summary: ${epps:-N/A}"
}

read_pkg_temp_millic() {
  local z
  for z in /sys/class/thermal/thermal_zone*; do
    [[ -f "${z}/type" && -f "${z}/temp" ]] || continue
    if [[ "$(<"${z}/type")" == "x86_pkg_temp" ]]; then
      cat "${z}/temp"
      return 0
    fi
  done
  echo ""
}

status_cmd() {
  info "=== CPU pstate ==="
  if [[ -d "${INTEL_PSTATE_DIR}" ]]; then
    for f in status no_turbo min_perf_pct max_perf_pct; do
      [[ -f "${INTEL_PSTATE_DIR}/${f}" ]] && info "${f}: $(<"${INTEL_PSTATE_DIR}/${f}")"
    done
  else
    warn "intel_pstate 디렉토리를 찾지 못했습니다."
  fi

  info "=== CPU policy ==="
  print_cpu_policy_summary
  [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq ]] && info "scaling_min_freq(cpu0): $(</sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq)"
  [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq ]] && info "scaling_max_freq(cpu0): $(</sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq)"
  [[ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq ]] && info "scaling_cur_freq(cpu0): $(</sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)"

  info "=== Thermal throttle counters ==="
  info "core_throttle_sum: $(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/core_throttle_count')"
  info "package_throttle_sum: $(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/package_throttle_count')"

  local pkg_temp
  pkg_temp="$(read_pkg_temp_millic)"
  [[ -n "${pkg_temp}" ]] && info "x86_pkg_temp: ${pkg_temp} mC"

  if command -v nvidia-smi >/dev/null 2>&1; then
    info "=== GPU ==="
    local gpu_line
    gpu_line="$(nvidia-smi --query-gpu=name,temperature.gpu,utilization.gpu,power.draw,pstate,clocks_throttle_reasons.sw_power_cap,clocks_throttle_reasons.hw_thermal_slowdown --format=csv,noheader 2>&1 || true)"
    if [[ -n "${gpu_line}" && "${gpu_line}" != Failed\ to\ initialize\ NVML:* ]]; then
      echo "${gpu_line}"
    fi
  fi

  info "=== Backup Files ==="
  if [[ -s "${ORIGINAL_BACKUP_FILE}" ]]; then
    info "original backup: ${ORIGINAL_BACKUP_FILE} (exists)"
  else
    info "original backup: ${ORIGINAL_BACKUP_FILE} (not found)"
  fi
  if [[ -s "${LAST_APPLY_SNAPSHOT_FILE}" ]]; then
    info "last apply snapshot: ${LAST_APPLY_SNAPSHOT_FILE} (exists)"
  else
    info "last apply snapshot: ${LAST_APPLY_SNAPSHOT_FILE} (not found)"
  fi
}

write_snapshot() {
  local snapshot_file="$1"
  : > "${snapshot_file}"
  echo "# voxelnext system perf guard snapshot" >> "${snapshot_file}"

  local f
  for f in \
    "${INTEL_PSTATE_DIR}/no_turbo" \
    "${INTEL_PSTATE_DIR}/min_perf_pct" \
    "${INTEL_PSTATE_DIR}/max_perf_pct"; do
    [[ -f "${f}" ]] && echo "FILE:${f}=$(<"${f}")" >> "${snapshot_file}"
  done

  for f in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
    [[ -f "${f}" ]] && echo "FILE:${f}=$(<"${f}")" >> "${snapshot_file}"
  done
  while IFS= read -r f; do
    [[ -f "${f}" ]] && echo "FILE:${f}=$(<"${f}")" >> "${snapshot_file}"
  done < <(list_epp_files)
}

capture_original_backup_once() {
  if [[ -s "${ORIGINAL_BACKUP_FILE}" ]]; then
    info "원본 백업 유지 (overwrite skipped): ${ORIGINAL_BACKUP_FILE}"
    return 0
  fi
  write_snapshot "${ORIGINAL_BACKUP_FILE}"
  info "원본 백업 저장: ${ORIGINAL_BACKUP_FILE}"
}

restore_from_snapshot() {
  local snapshot_file="$1"
  [[ -f "${snapshot_file}" ]] || { err "백업 파일이 없습니다: ${snapshot_file}"; exit 1; }
  while IFS= read -r line; do
    [[ "${line}" == FILE:* ]] || continue
    local kv path value
    kv="${line#FILE:}"
    path="${kv%%=*}"
    value="${kv#*=}"
    if [[ -w "${path}" ]]; then
      printf "%s" "${value}" > "${path}" || true
    fi
  done < "${snapshot_file}"
}

apply_cmd() {
  need_root
  local max_perf_pct="${1:-70}"

  if [[ ! -d "${INTEL_PSTATE_DIR}" ]]; then
    err "intel_pstate를 찾을 수 없습니다. 이 스크립트는 intel_pstate 기반 시스템을 대상으로 합니다."
    exit 1
  fi

  if ! [[ "${max_perf_pct}" =~ ^[0-9]+$ ]] || (( max_perf_pct < 30 || max_perf_pct > 100 )); then
    err "max_perf_pct는 30~100 정수로 지정하세요."
    exit 1
  fi

  capture_original_backup_once
  write_snapshot "${LAST_APPLY_SNAPSHOT_FILE}"
  info "직전 상태 스냅샷 저장: ${LAST_APPLY_SNAPSHOT_FILE}"

  # 1) Turbo off: avoid aggressive short boosts and thermal spikes
  if [[ -w "${INTEL_PSTATE_DIR}/no_turbo" ]]; then
    echo 1 > "${INTEL_PSTATE_DIR}/no_turbo"
  fi

  # 2) Cap peak performance percentage
  if [[ -w "${INTEL_PSTATE_DIR}/max_perf_pct" ]]; then
    echo "${max_perf_pct}" > "${INTEL_PSTATE_DIR}/max_perf_pct"
  fi

  # 3) Move governor to powersave for smoother sustained behavior
  local g
  for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
    [[ -w "${g}" ]] && echo powersave > "${g}" || true
  done

  # 4) Prefer balanced power/perf policy when available
  local e
  while IFS= read -r e; do
    [[ -w "${e}" ]] && echo balance_power > "${e}" || true
  done < <(list_epp_files)

  info "적용 완료 (max_perf_pct=${max_perf_pct}, no_turbo=1, governor=powersave, EPP=balance_power)"
  status_cmd
}

revert_cmd() {
  need_root
  restore_from_snapshot "${ORIGINAL_BACKUP_FILE}"
  rm -f "${ORIGINAL_BACKUP_FILE}" "${LAST_APPLY_SNAPSHOT_FILE}"
  info "복구 완료 (원본 백업/직전 스냅샷 정리됨)"
  status_cmd
}

watch_cmd() {
  local interval="${1:-1}"
  local count="${2:-0}"
  local i=0

  local core_prev pkg_prev
  core_prev="$(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/core_throttle_count')"
  pkg_prev="$(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/package_throttle_count')"

  echo "ts,pkg_temp_mC,cpu0_cur_khz,load1,core_thr_delta,pkg_thr_delta,gpu_temp_C,gpu_util_pct,gpu_power_W,gpu_sw_power_cap"
  while true; do
    local ts pkg_temp cur_khz load1 core_now pkg_now dcore dpkg
    ts="$(date +%s)"
    pkg_temp="$(read_pkg_temp_millic)"
    cur_khz="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo '')"
    load1="$(awk '{print $1}' /proc/loadavg)"
    core_now="$(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/core_throttle_count')"
    pkg_now="$(sum_throttle_counts '/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/package_throttle_count')"
    dcore=$((core_now - core_prev))
    dpkg=$((pkg_now - pkg_prev))
    core_prev="${core_now}"
    pkg_prev="${pkg_now}"

    local gpu_temp="" gpu_util="" gpu_power="" gpu_sw_cap=""
    if command -v nvidia-smi >/dev/null 2>&1; then
      local line
      line="$(nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,power.draw,clocks_throttle_reasons.sw_power_cap --format=csv,noheader,nounits 2>/dev/null || true)"
      if [[ -n "${line}" ]]; then
        IFS=',' read -r gpu_temp gpu_util gpu_power gpu_sw_cap <<< "${line}"
        gpu_temp="${gpu_temp// /}"
        gpu_util="${gpu_util// /}"
        gpu_power="${gpu_power// /}"
        gpu_sw_cap="${gpu_sw_cap// /}"
      fi
    fi

    echo "${ts},${pkg_temp},${cur_khz},${load1},${dcore},${dpkg},${gpu_temp},${gpu_util},${gpu_power},${gpu_sw_cap}"
    i=$((i + 1))
    if (( count > 0 && i >= count )); then
      break
    fi
    sleep "${interval}"
  done
}

usage() {
  cat <<'EOF'
Usage:
  system_perf_guard.sh status
  system_perf_guard.sh apply [max_perf_pct]
  system_perf_guard.sh revert
  system_perf_guard.sh watch [interval_sec] [count]

Backup behavior:
  - First apply in a cycle captures ORIGINAL baseline once.
  - Repeated apply does NOT overwrite ORIGINAL baseline.
  - revert restores ORIGINAL baseline and then removes backup files.

Examples:
  ./system_perf_guard.sh status
  sudo ./system_perf_guard.sh apply 70
  sudo ./system_perf_guard.sh revert
  ./system_perf_guard.sh watch 1 60
EOF
}

main() {
  local cmd="${1:-}"
  case "${cmd}" in
    status)
      status_cmd
      ;;
    apply)
      apply_cmd "${2:-70}"
      ;;
    revert)
      revert_cmd
      ;;
    watch)
      watch_cmd "${2:-1}" "${3:-0}"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
