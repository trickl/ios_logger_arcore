#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/pull_latest_dataset.sh [options]

Options:
  --serial <id>           ADB serial to use (auto-detect if omitted)
  --dest-root <path>      Local destination root (default: ./pulled_datasets)
  --src-root <path>       Device dataset root
                          (default: /sdcard/Android/data/com.trickl.iosloggerarcore/files/datasets)
  --dataset <name>        Pull specific dataset folder name (default: latest)
  --label <name>          Also copy dataset under ./pulled_datasets/labeled/<dataset>__<name>
  --sync                  Run sync-data.py after pull (if found)
  --sync-script <path>    Path to sync script (default: ../ios_logger/sync-data.py)
  --quality-check         Run scripts/validate_capture_quality.py after pull (default: on)
  --no-quality-check      Disable post-pull quality check
  --no-validate           Skip post-pull validation output
  -h, --help              Show this help

Examples:
  scripts/pull_latest_dataset.sh
  scripts/pull_latest_dataset.sh --sync
  scripts/pull_latest_dataset.sh --serial 192.168.1.66:46867 --dataset 2026-03-01T11-27-44
EOF
}

SERIAL=""
DEST_ROOT="$(pwd)/pulled_datasets"
SRC_ROOT="/sdcard/Android/data/com.trickl.iosloggerarcore/files/datasets"
DATASET=""
DO_SYNC=0
VALIDATE=1
QUALITY_CHECK=1
LABEL=""
SYNC_SCRIPT="$(pwd)/../ios_logger/sync-data.py"
QUALITY_SCRIPT="$(pwd)/scripts/validate_capture_quality.py"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial)
      SERIAL="$2"
      shift 2
      ;;
    --dest-root)
      DEST_ROOT="$2"
      shift 2
      ;;
    --src-root)
      SRC_ROOT="$2"
      shift 2
      ;;
    --dataset)
      DATASET="$2"
      shift 2
      ;;
    --label)
      LABEL="$2"
      shift 2
      ;;
    --sync)
      DO_SYNC=1
      shift
      ;;
    --sync-script)
      SYNC_SCRIPT="$2"
      shift 2
      ;;
    --quality-check)
      QUALITY_CHECK=1
      shift
      ;;
    --no-quality-check)
      QUALITY_CHECK=0
      shift
      ;;
    --no-validate)
      VALIDATE=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$SERIAL" ]]; then
  mapfile -t DEVICES < <(adb devices | awk 'NR>1 && $2=="device" {print $1}')
  if [[ ${#DEVICES[@]} -eq 0 ]]; then
    echo "ERROR: no connected adb devices found" >&2
    exit 1
  fi

  SERIAL="${DEVICES[0]}"
  if [[ ${#DEVICES[@]} -gt 1 ]]; then
    echo "[pull] multiple adb devices found; defaulting to first: $SERIAL" >&2
    printf '[pull] available devices: %s\n' "${DEVICES[*]}" >&2
    echo "[pull] pass --serial to override" >&2
  fi
fi

if [[ -z "$DATASET" ]]; then
  DATASET="$(adb -s "$SERIAL" shell "ls -1 $SRC_ROOT" | tr -d '\r' | sort | tail -n 1)"
fi

if [[ -z "$DATASET" ]]; then
  echo "ERROR: could not determine dataset name from device root: $SRC_ROOT" >&2
  exit 1
fi

SRC_PATH="$SRC_ROOT/$DATASET"
mkdir -p "$DEST_ROOT"
rm -rf "$DEST_ROOT/$DATASET"

echo "[pull] serial=$SERIAL"
echo "[pull] src=$SRC_PATH"
echo "[pull] dst=$DEST_ROOT/$DATASET"

adb -s "$SERIAL" pull "$SRC_PATH" "$DEST_ROOT/" >/tmp/ios_logger_arcore_pull.log

grep -E "files pulled|bytes" /tmp/ios_logger_arcore_pull.log | tail -n 1 || true

LOCAL_DATASET="$DEST_ROOT/$DATASET"

if [[ ! -d "$LOCAL_DATASET" ]]; then
  echo "ERROR: pull appears to have failed, missing local dataset folder: $LOCAL_DATASET" >&2
  exit 1
fi

if [[ "$VALIDATE" -eq 1 ]]; then
  echo
  echo "[validate] required files"
  REQUIRED=(
    Frames.m4v Frames.txt ARposes.txt
    Accel.txt Gyro.txt GPS.txt Head.txt Motion.txt MotARH.txt MotMagnFull.txt Magnet.txt
  )

  missing=0
  for f in "${REQUIRED[@]}"; do
    if [[ -f "$LOCAL_DATASET/$f" ]]; then
      echo "  OK  $f"
    else
      echo "  MISSING  $f"
      missing=1
    fi
  done

  echo
  echo "[validate] text line counts"
  for f in "$LOCAL_DATASET"/*.txt; do
    b="$(basename "$f")"
    c="$(wc -l < "$f")"
    echo "  $b:$c"
  done | sort

  if [[ -f "$LOCAL_DATASET/ARposes.txt" ]]; then
    echo
    echo "[validate] ARposes monotonic timestamps"
    awk -F, '
      BEGIN { bad=0; eq=0; prev="" }
      {
        t=$1+0
        if (prev!="") {
          if (t < prev) bad++
          if (t == prev) eq++
        }
        prev=t
      }
      END {
        printf("  rows=%d non_increasing=%d equal_neighbors=%d\n", NR, bad+eq, eq)
      }
    ' "$LOCAL_DATASET/ARposes.txt"
  fi

  if [[ -f "$LOCAL_DATASET/Frames.txt" ]]; then
    echo
    echo "[validate] Frames intrinsics mean"
    awk -F, '
      BEGIN { n=0; sumfx=0; sumfy=0; sumcx=0; sumcy=0 }
      NF>=6 {
        sumfx+=$3; sumfy+=$4; sumcx+=$5; sumcy+=$6; n++
      }
      END {
        if (n>0) {
          printf("  rows=%d mean_fx=%.3f mean_fy=%.3f mean_cx=%.3f mean_cy=%.3f\n", n, sumfx/n, sumfy/n, sumcx/n, sumcy/n)
        } else {
          print "  no intrinsics rows"
        }
      }
    ' "$LOCAL_DATASET/Frames.txt"
  fi

  if [[ "$missing" -ne 0 ]]; then
    echo
    echo "[validate] FAILED: one or more required files are missing" >&2
    exit 1
  fi
fi

if [[ "$DO_SYNC" -eq 1 ]]; then
  echo
  echo "[sync] running: $SYNC_SCRIPT $LOCAL_DATASET"
  if [[ ! -f "$SYNC_SCRIPT" ]]; then
    echo "ERROR: sync script not found: $SYNC_SCRIPT" >&2
    exit 1
  fi

  if command -v python3 >/dev/null 2>&1; then
    python3 "$SYNC_SCRIPT" "$LOCAL_DATASET"
  elif command -v python >/dev/null 2>&1; then
    python "$SYNC_SCRIPT" "$LOCAL_DATASET"
  else
    echo "ERROR: python interpreter not found for sync step" >&2
    exit 1
  fi
fi

if [[ "$QUALITY_CHECK" -eq 1 ]]; then
  echo
  echo "[quality] running: $QUALITY_SCRIPT --dataset $LOCAL_DATASET"
  if [[ ! -f "$QUALITY_SCRIPT" ]]; then
    echo "ERROR: quality script not found: $QUALITY_SCRIPT" >&2
    exit 1
  fi

  if command -v python3 >/dev/null 2>&1; then
    python3 "$QUALITY_SCRIPT" --dataset "$LOCAL_DATASET"
  elif command -v python >/dev/null 2>&1; then
    python "$QUALITY_SCRIPT" --dataset "$LOCAL_DATASET"
  else
    echo "ERROR: python interpreter not found for quality check" >&2
    exit 1
  fi
fi

if [[ -n "$LABEL" ]]; then
  sanitized_label="$(echo "$LABEL" | tr ' /' '__' | tr -cd '[:alnum:]_.-')"
  if [[ -z "$sanitized_label" ]]; then
    echo "ERROR: --label produced empty sanitized value; choose a label with letters/numbers" >&2
    exit 1
  fi

  label_root="$DEST_ROOT/labeled"
  mkdir -p "$label_root"
  labeled_path="$label_root/${DATASET}__${sanitized_label}"
  rm -rf "$labeled_path"
  cp -a "$LOCAL_DATASET" "$labeled_path"
  echo "[label] created: $labeled_path"
fi

echo
echo "Done. Local dataset: $LOCAL_DATASET"
