#!/usr/bin/env bash
set -u

OUT_DIR="${1:-/tmp/fast_livo2_rviz_perf_$(date +%Y%m%d_%H%M%S)}"
POINT_TOPIC="${POINT_TOPIC:-/cloud_registered}"
ODOM_TOPIC="${ODOM_TOPIC:-/aft_mapped_to_init}"
SAMPLES="${SAMPLES:-10 30 60 180 300 600}"

mkdir -p "$OUT_DIR"

CSV="$OUT_DIR/timeline.csv"
printf "elapsed_s,manual_rviz_fps,rviz_pid,rviz_cpu_percent,rviz_rss_kb,cloud_hz,cloud_bw,cloud_height,cloud_width,cloud_point_step,cloud_row_step,cloud_points,cloud_frame_bytes,odom_hz,cpu_freq_khz,thermal_c,temp_notes\n" > "$CSV"

collect_cpu_freqs() {
  local values=()
  for p in /sys/devices/system/cpu/cpufreq/policy*; do
    [ -e "$p/scaling_cur_freq" ] || continue
    values+=("$(basename "$p"):$(cat "$p/scaling_cur_freq" 2>/dev/null)")
  done
  (IFS=';'; echo "${values[*]}")
}

collect_thermals() {
  local values=()
  for h in /sys/class/hwmon/hwmon*; do
    [ -d "$h" ] || continue
    for t in "$h"/temp*_input; do
      [ -e "$t" ] || continue
      local name raw temp
      name="$(basename "$h")/$(basename "$t")"
      raw="$(cat "$t" 2>/dev/null || true)"
      [ -n "$raw" ] || continue
      temp="$(awk "BEGIN { printf \"%.1f\", $raw / 1000.0 }")"
      values+=("$name:$temp")
    done
  done
  for z in /sys/class/thermal/thermal_zone*; do
    [ -e "$z/temp" ] || continue
    local type temp raw
    type="$(cat "$z/type" 2>/dev/null || basename "$z")"
    raw="$(cat "$z/temp" 2>/dev/null || true)"
    [ -n "$raw" ] || continue
    temp="$(awk "BEGIN { printf \"%.1f\", $raw / 1000.0 }")"
    values+=("$type:$temp")
  done
  (IFS=';'; echo "${values[*]}")
}

topic_hz() {
  timeout 8 ros2 topic hz "$1" 2>&1 | awk '/average rate:/ {rate=$3} END {print rate}'
}

topic_bw() {
  timeout 8 ros2 topic bw "$1" 2>&1 | awk '/average:/ {avg=$2" "$3} END {print avg}'
}

pointcloud_meta() {
  timeout 8 ros2 topic echo "$POINT_TOPIC" --once 2>/dev/null |
    awk '
      /^height:/ {height=$2}
      /^width:/ {width=$2}
      /^point_step:/ {point_step=$2}
      /^row_step:/ {row_step=$2}
      END {
        points = height * width
        bytes = height * row_step
        printf "%s,%s,%s,%s,%s,%s", height, width, point_step, row_step, points, bytes
      }'
}

{
  echo "date: $(date --iso-8601=seconds)"
  echo "DISPLAY: ${DISPLAY:-}"
  echo "XDG_SESSION_TYPE: ${XDG_SESSION_TYPE:-}"
  echo
  echo "glxinfo -B:"
  if command -v glxinfo >/dev/null 2>&1; then
    glxinfo -B 2>&1
  else
    echo "glxinfo not found; install mesa-utils to capture OpenGL renderer details."
  fi
  echo
  echo "CPU policies:"
  for p in /sys/devices/system/cpu/cpufreq/policy*; do
    [ -d "$p" ] || continue
    echo "=== $p ==="
    for f in scaling_governor scaling_cur_freq scaling_min_freq scaling_max_freq scaling_available_governors scaling_available_frequencies; do
      [ -e "$p/$f" ] && printf "%s: %s\n" "$f" "$(cat "$p/$f" 2>/dev/null)"
    done
  done
} > "$OUT_DIR/system_snapshot.txt"

START="$(date +%s)"
for target in $SAMPLES; do
  while :; do
    now="$(date +%s)"
    elapsed=$((now - START))
    [ "$elapsed" -ge "$target" ] && break
    sleep 1
  done

  rviz_pid="$(pgrep -n -x rviz2 || true)"
  rviz_cpu=""
  rviz_rss=""
  if [ -n "$rviz_pid" ]; then
    read -r rviz_cpu rviz_rss < <(ps -p "$rviz_pid" -o %cpu=,rss=)
  fi

  cloud_hz="$(topic_hz "$POINT_TOPIC")"
  cloud_bw="$(topic_bw "$POINT_TOPIC")"
  meta="$(pointcloud_meta)"
  odom_hz="$(topic_hz "$ODOM_TOPIC")"
  cpu_freq="$(collect_cpu_freqs)"
  thermal="$(collect_thermals)"

  printf "%s,,%s,%s,%s,%s,%s,%s,%s,%s,\n" \
    "$target" "$rviz_pid" "$rviz_cpu" "$rviz_rss" "$cloud_hz" "$cloud_bw" \
    "$meta" "$odom_hz" "$cpu_freq" "$thermal" >> "$CSV"
done

echo "Wrote $CSV"
echo "Fill manual_rviz_fps from the RViz lower-left FPS value at each sample time."
