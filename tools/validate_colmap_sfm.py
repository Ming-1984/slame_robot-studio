#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO


COLMAP_CAMERA_MODEL_NUM_PARAMS = {
    0: ("SIMPLE_PINHOLE", 3),
    1: ("PINHOLE", 4),
    2: ("SIMPLE_RADIAL", 4),
    3: ("RADIAL", 5),
    4: ("OPENCV", 8),
    5: ("OPENCV_FISHEYE", 8),
    6: ("FULL_OPENCV", 12),
    7: ("FOV", 5),
    8: ("SIMPLE_RADIAL_FISHEYE", 4),
    9: ("RADIAL_FISHEYE", 5),
    10: ("THIN_PRISM_FISHEYE", 12),
}


class BinaryReader:
    def __init__(self, fh: BinaryIO, path: Path) -> None:
        self._fh = fh
        self._path = path

    def read_exact(self, size: int) -> bytes:
        buf = self._fh.read(size)
        if len(buf) != size:
            raise EOFError(f"Unexpected EOF while reading {self._path} (need {size}, got {len(buf)})")
        return buf

    def unpack(self, fmt: str):
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, self.read_exact(size))

    def read_cstring(self) -> str:
        out = bytearray()
        while True:
            ch = self._fh.read(1)
            if not ch:
                raise EOFError(f"Unexpected EOF while reading C-string in {self._path}")
            if ch == b"\x00":
                return out.decode("utf-8", errors="replace")
            out.extend(ch)

    def seek(self, offset: int, whence: int = os.SEEK_CUR) -> None:
        self._fh.seek(offset, whence)


@dataclass(frozen=True)
class Camera:
    camera_id: int
    model_id: int
    model_name: str
    width: int
    height: int
    params: tuple[float, ...]


@dataclass(frozen=True)
class ImageEntry:
    image_id: int
    camera_id: int
    name: str
    num_points2d: int


@dataclass(frozen=True)
class PointsSummary:
    num_points3d: int
    invalid_track_refs: int


def read_cameras_bin(path: Path) -> list[Camera]:
    with path.open("rb") as fh:
        br = BinaryReader(fh, path)
        (num_cameras,) = br.unpack("<Q")
        cameras: list[Camera] = []
        for _ in range(num_cameras):
            camera_id, model_id = br.unpack("<ii")
            (width, height) = br.unpack("<QQ")
            model_name, num_params = COLMAP_CAMERA_MODEL_NUM_PARAMS.get(model_id, (f"UNKNOWN({model_id})", None))
            if num_params is None:
                raise ValueError(f"Unsupported/unknown camera model_id={model_id} in {path}")
            params = br.unpack("<" + "d" * num_params)
            cameras.append(
                Camera(
                    camera_id=camera_id,
                    model_id=model_id,
                    model_name=model_name,
                    width=width,
                    height=height,
                    params=tuple(float(x) for x in params),
                )
            )
        return cameras


def read_images_bin(path: Path) -> tuple[list[ImageEntry], int]:
    with path.open("rb") as fh:
        br = BinaryReader(fh, path)
        (num_images,) = br.unpack("<Q")
        images: list[ImageEntry] = []
        total_points2d = 0
        for _ in range(num_images):
            (image_id,) = br.unpack("<i")
            br.read_exact(32)  # qvec (4 doubles)
            br.read_exact(24)  # tvec (3 doubles)
            (camera_id,) = br.unpack("<i")
            name = br.read_cstring()
            (num_points2d,) = br.unpack("<Q")
            total_points2d += num_points2d
            images.append(
                ImageEntry(
                    image_id=image_id,
                    camera_id=camera_id,
                    name=name,
                    num_points2d=num_points2d,
                )
            )
            br.seek(num_points2d * 24, os.SEEK_CUR)  # x(double), y(double), point3D_id(int64)
        return images, total_points2d


def read_points3d_bin(
    path: Path,
    image_points2d_counts: dict[int, int] | None,
    check_tracks: bool,
) -> PointsSummary:
    with path.open("rb") as fh:
        br = BinaryReader(fh, path)
        (num_points3d,) = br.unpack("<Q")
        invalid_track_refs = 0
        for _ in range(num_points3d):
            br.read_exact(8)  # point3D_id (int64)
            br.read_exact(24)  # xyz (3 doubles)
            br.read_exact(3)  # rgb (3 bytes)
            br.read_exact(8)  # error (double)
            (track_len,) = br.unpack("<Q")
            if check_tracks and image_points2d_counts is not None:
                track_bytes = br.read_exact(track_len * 8)
                for i in range(track_len):
                    image_id, point2d_idx = struct.unpack_from("<ii", track_bytes, i * 8)
                    max_points2d = image_points2d_counts.get(image_id)
                    if max_points2d is None or point2d_idx < 0 or point2d_idx >= max_points2d:
                        invalid_track_refs += 1
            else:
                br.seek(track_len * 8, os.SEEK_CUR)
        return PointsSummary(num_points3d=num_points3d, invalid_track_refs=invalid_track_refs)


def find_sparse_models(dataset_root: Path) -> list[Path]:
    sparse_dir = dataset_root / "sparse"
    if not sparse_dir.exists():
        return []

    models: list[Path] = []
    if (sparse_dir / "cameras.bin").exists():
        models.append(sparse_dir)

    for child in sorted(sparse_dir.iterdir()):
        if child.is_dir() and (child / "cameras.bin").exists():
            models.append(child)

    # de-dup
    seen = set()
    uniq: list[Path] = []
    for m in models:
        if m not in seen:
            uniq.append(m)
            seen.add(m)
    return uniq


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate/inspect a COLMAP SfM dataset (images/ + sparse/*.bin).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "dataset_root",
        nargs="?",
        default=".",
        help="Dataset root directory (contains images/ and sparse/).",
    )
    parser.add_argument(
        "--image-folder",
        default="images",
        help="Image folder name under dataset_root.",
    )
    parser.add_argument(
        "--check-tracks",
        action="store_true",
        help="Validate points3D track references against images.bin point2D counts.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero if any missing images or invalid track references are found.",
    )
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root).expanduser().resolve()
    if not dataset_root.exists():
        print(f"ERROR: dataset_root not found: {dataset_root}", file=sys.stderr)
        return 2

    image_dir = dataset_root / args.image_folder
    if not image_dir.exists():
        print(f"ERROR: images folder not found: {image_dir}", file=sys.stderr)
        return 2

    models = find_sparse_models(dataset_root)
    if not models:
        print(f"ERROR: no sparse model found under: {dataset_root / 'sparse'}", file=sys.stderr)
        return 2

    print(f"Dataset: {dataset_root}")
    print(f"Image folder: {image_dir}")
    print(f"Sparse models: {', '.join(str(m.relative_to(dataset_root)) for m in models)}")

    overall_missing_images = 0
    overall_invalid_track_refs = 0
    for model_dir in models:
        cameras_path = model_dir / "cameras.bin"
        images_path = model_dir / "images.bin"
        points_path = model_dir / "points3D.bin"
        for required in (cameras_path, images_path, points_path):
            if not required.exists():
                print(f"ERROR: missing required file: {required}", file=sys.stderr)
                return 2

        try:
            cameras = read_cameras_bin(cameras_path)
            images, total_points2d = read_images_bin(images_path)
            image_points2d_counts = {img.image_id: img.num_points2d for img in images}
            points_summary = read_points3d_bin(
                points_path,
                image_points2d_counts=image_points2d_counts,
                check_tracks=bool(args.check_tracks),
            )
        except Exception as exc:
            print(f"ERROR: failed to parse sparse model under {model_dir}: {exc}", file=sys.stderr)
            return 3

        print("")
        print(f"Model: {model_dir.relative_to(dataset_root)}")
        print(f"  Cameras: {len(cameras)}")
        for cam in cameras:
            print(
                "  - "
                + f"id={cam.camera_id} model={cam.model_name} (id={cam.model_id}) "
                + f"w={cam.width} h={cam.height} params={tuple(round(p, 6) for p in cam.params)}"
            )

        missing = 0
        for img in images:
            if not (image_dir / img.name).exists():
                missing += 1
        overall_missing_images += missing

        print(f"  Images: {len(images)} (missing files: {missing})")
        print(f"  Points2D total: {total_points2d}")
        print(f"  Points3D: {points_summary.num_points3d}")
        if args.check_tracks:
            print(f"  Invalid points3D track refs: {points_summary.invalid_track_refs}")
            overall_invalid_track_refs += points_summary.invalid_track_refs

    if args.strict and (overall_missing_images or overall_invalid_track_refs):
        print(
            f"ERROR: strict validation failed (missing_images={overall_missing_images}, invalid_track_refs={overall_invalid_track_refs})",
            file=sys.stderr,
        )
        return 4

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
