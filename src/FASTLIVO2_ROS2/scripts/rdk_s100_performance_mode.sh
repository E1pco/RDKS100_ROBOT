#!/usr/bin/env bash
set -euo pipefail

STATE_FILE="${STATE_FILE:-/tmp/rdk_s100_cpu_governors.prev}"

usage() {
  cat <<'EOF'
Usage:
  rdk_s100_performance_mode.sh status
  rdk_s100_performance_mode.sh performance
  rdk_s100_performance_mode.sh restore
  rdk_s100_performance_mode.sh monitor

Commands:
  status       Print current CPU governors, frequencies, temperatures, and hrut_somstatus if available.
  performance  Save current governors, set CPU governor to performance, then verify.
  restore      Restore governors saved by the last performance command.
  monitor      Refresh current frequencies and temperatures every 0.5 s.

Notes:
  - Based on the official RDK S100 frequency-management sysfs paths.
  - Runtime settings are not persistent across reboot.
EOF
}

require_root_for_write() {
  if [[ "${EUID}" -ne 0 ]]; then
    exec sudo --preserve-env=STATE_FILE "$0" "$@"
  fi
}

cpu_governor_files() {
  local seen=""
  for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor \
           /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do
    [[ -e "$f" ]] || continue
    case "$seen" in
      *"|$f|"*) ;;
      *)
        seen="${seen}|${f}|"
        printf '%s\n' "$f"
        ;;
    esac
  done
}

print_cpu_status() {
  echo "== CPU frequency policies =="
  for p in /sys/devices/system/cpu/cpufreq/policy*; do
    [[ -d "$p" ]] || continue
    echo "-- $p"
    for f in affected_cpus scaling_governor scaling_cur_freq cpuinfo_cur_freq scaling_min_freq scaling_max_freq scaling_available_governors scaling_available_frequencies; do
      [[ -e "$p/$f" ]] && printf "%-31s %s\n" "$f:" "$(cat "$p/$f" 2>/dev/null || true)"
    done
  done

  echo
  echo "== CPU governor files =="
  while IFS= read -r f; do
    printf "%s: %s\n" "$f" "$(cat "$f" 2>/dev/null || true)"
  done < <(cpu_governor_files)
}

print_temperatures() {
  echo
  echo "== RDK S100 hwmon temperatures =="
  for f in /sys/class/hwmon/hwmon0/temp*_input; do
    [[ -e "$f" ]] || continue
    local raw c
    raw="$(cat "$f" 2>/dev/null || true)"
    [[ -n "$raw" ]] || continue
    c="$(awk "BEGIN { printf \"%.3f\", ${raw} / 1000.0 }")"
    printf "%s: %s C\n" "$f" "$c"
  done

  echo
  echo "== Thermal zones =="
  for z in /sys/class/thermal/thermal_zone*; do
    [[ -d "$z" ]] || continue
    local type policy temp c
    type="$(cat "$z/type" 2>/dev/null || basename "$z")"
    policy="$(cat "$z/policy" 2>/dev/null || true)"
    temp="$(cat "$z/temp" 2>/dev/null || true)"
    if [[ -n "$temp" ]]; then
      c="$(awk "BEGIN { printf \"%.3f\", ${temp} / 1000.0 }")"
      printf "%s type=%s policy=%s temp=%s C\n" "$z" "$type" "$policy" "$c"
    else
      printf "%s type=%s policy=%s\n" "$z" "$type" "$policy"
    fi
  done
}

print_hrut_status() {
  if command -v hrut_somstatus >/dev/null 2>&1; then
    echo
    echo "== hrut_somstatus =="
    sudo hrut_somstatus || true
  fi
}

status() {
  print_cpu_status
  print_temperatures
  print_hrut_status
}

set_performance() {
  require_root_for_write performance

  : > "$STATE_FILE"
  while IFS= read -r f; do
    [[ -w "$f" ]] || continue
    printf "%s=%s\n" "$f" "$(cat "$f" 2>/dev/null || true)" >> "$STATE_FILE"
  done < <(cpu_governor_files)

  if [[ ! -s "$STATE_FILE" ]]; then
    echo "No writable CPU governor files found." >&2
    exit 1
  fi

  echo "Saved previous governors to $STATE_FILE"
  while IFS= read -r f; do
    [[ -w "$f" ]] || continue
    echo performance > "$f"
  done < <(cpu_governor_files)

  echo "Set CPU governor to performance."
  echo
  status
}

restore() {
  require_root_for_write restore

  if [[ ! -s "$STATE_FILE" ]]; then
    echo "No saved governor state at $STATE_FILE" >&2
    exit 1
  fi

  while IFS='=' read -r f governor; do
    [[ -n "${f:-}" && -e "$f" && -w "$f" ]] || continue
    echo "$governor" > "$f"
  done < "$STATE_FILE"

  echo "Restored governors from $STATE_FILE"
  echo
  status
}

monitor() {
  while true; do
    clear
    date --iso-8601=seconds
    echo
    for p in /sys/devices/system/cpu/cpufreq/policy*; do
      [[ -d "$p" ]] || continue
      printf "%s governor=%s cur=%s min=%s max=%s\n" \
        "$p" \
        "$(cat "$p/scaling_governor" 2>/dev/null || true)" \
        "$(cat "$p/scaling_cur_freq" 2>/dev/null || true)" \
        "$(cat "$p/scaling_min_freq" 2>/dev/null || true)" \
        "$(cat "$p/scaling_max_freq" 2>/dev/null || true)"
    done
    echo
    for f in /sys/class/hwmon/hwmon0/temp*_input; do
      [[ -e "$f" ]] || continue
      raw="$(cat "$f" 2>/dev/null || true)"
      [[ -n "$raw" ]] && printf "%s %.3f C\n" "$f" "$(awk "BEGIN { print ${raw} / 1000.0 }")"
    done
    sleep 0.5
  done
}

cmd="${1:-status}"
case "$cmd" in
  status) status ;;
  performance) set_performance ;;
  restore) restore ;;
  monitor) monitor ;;
  -h|--help|help) usage ;;
  *)
    usage >&2
    exit 2
    ;;
esac
