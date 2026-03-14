#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/fetch_archived_lfs_data.sh [--snapshot-dir DIR] [--no-snapshot]

Fetches archived telemetry/events/Livox data from Git LFS, hydrates files in the
working tree, and optionally stores a timestamped snapshot under archive/.

Options:
  --snapshot-dir DIR  Custom output directory for copied artifacts
  --no-snapshot       Fetch + hydrate only, skip archive copy
  -h, --help          Show this help
EOF
}

is_lfs_pointer() {
  local file="$1"
  [[ -f "$file" ]] || return 1
  # Use grep in text mode (-a) to avoid shell warnings on binary files.
  grep -a -m1 -qx "version https://git-lfs.github.com/spec/v1" "$file" 2>/dev/null
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Required command not found: $1" >&2
    exit 1
  fi
}

snapshot_enabled=1
snapshot_dir=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --snapshot-dir)
      snapshot_dir="${2:-}"
      if [[ -z "$snapshot_dir" ]]; then
        echo "[ERROR] --snapshot-dir requires a value" >&2
        exit 1
      fi
      shift 2
      ;;
    --no-snapshot)
      snapshot_enabled=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

require_cmd git
require_cmd git-lfs

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$repo_root" ]]; then
  echo "[ERROR] Not inside a git repository" >&2
  exit 1
fi

cd "$repo_root"

timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$snapshot_dir" ]]; then
  snapshot_dir="archive/lfs_snapshots/$timestamp"
fi

include_patterns=(
  "data/telemetry/telemetry_*.csv"
  "data/events/events_*.txt"
  "data/livox/livox_imu_*.csv"
  "data/livox/livox_pointcloud_*.bin"
  "telemetry_*.csv"
  "events_*.txt"
  "livox_imu_*.csv"
  "livox_pointcloud_*.bin"
)

include_csv="$(IFS=, ; echo "${include_patterns[*]}")"

echo "[INFO] Fetching archived data objects from Git LFS..."
git lfs fetch --include="$include_csv"

echo "[INFO] Hydrating working tree files from LFS cache..."
git lfs pull --include="$include_csv"

mapfile -t matched_files < <(
  {
    for pat in "${include_patterns[@]}"; do
      compgen -G "$pat" || true
    done
  } | sort -u
)

hydrated=0
pointers=0
for f in "${matched_files[@]}"; do
  if is_lfs_pointer "$f"; then
    ((pointers+=1))
  else
    ((hydrated+=1))
  fi
done

echo "[INFO] Files matched: ${#matched_files[@]}"
echo "[INFO] Hydrated files: $hydrated"
echo "[INFO] Remaining LFS pointers: $pointers"

if [[ "$snapshot_enabled" -eq 1 ]]; then
  mkdir -p "$snapshot_dir"

  for f in "${matched_files[@]}"; do
    [[ -f "$f" ]] || continue
    if is_lfs_pointer "$f"; then
      continue
    fi
    mkdir -p "$snapshot_dir/$(dirname "$f")"
    cp -f "$f" "$snapshot_dir/$f"
  done

  manifest="$snapshot_dir/manifest.txt"
  {
    echo "ASCEND LFS Archive Snapshot"
    echo "Generated: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Repository: $repo_root"
    echo "Hydrated files: $hydrated"
    echo "Remaining pointers: $pointers"
    echo ""
    echo "Included patterns:"
    for pat in "${include_patterns[@]}"; do
      echo "- $pat"
    done
  } > "$manifest"

  echo "[INFO] Snapshot stored at: $snapshot_dir"
  echo "[INFO] Manifest written: $manifest"
else
  echo "[INFO] Snapshot disabled (--no-snapshot)."
fi

if [[ "$pointers" -gt 0 ]]; then
  echo "[WARN] Some files are still pointers. Confirm they exist in remote LFS storage."
fi

echo "[DONE] LFS archive fetch complete."
