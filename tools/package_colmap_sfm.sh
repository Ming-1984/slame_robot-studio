#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <sfm_dataset_dir> [out_dir] [name_prefix]" >&2
  echo "Example: $0 aurora_remote-release-2.1.0-rc2/densified_map/sfm_db_1 /tmp sfm_db_1_backup" >&2
  exit 2
fi

sfm_dir="$1"
out_dir="${2:-.}"
name_prefix="${3:-}"

if [[ ! -d "$sfm_dir" ]]; then
  echo "ERROR: not a directory: $sfm_dir" >&2
  exit 2
fi

mkdir -p "$out_dir"

base_name="$(basename "$sfm_dir")"
timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -n "$name_prefix" ]]; then
  archive_name="${name_prefix}_${timestamp}"
else
  archive_name="${base_name}_${timestamp}"
fi

tarball="${out_dir%/}/${archive_name}.tar.gz"

parent_dir="$(cd "$(dirname "$sfm_dir")" && pwd)"
tar -C "$parent_dir" -czf "$tarball" "$base_name"

sha256sum "$tarball" > "${tarball}.sha256"

echo "OK: $tarball"
echo "OK: ${tarball}.sha256"

