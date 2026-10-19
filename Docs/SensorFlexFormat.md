# SensorFlex Session Format (SFZ)

Version **2.0**. A self-contained `.sfz` archive (ZIP) holding one AR session — color video,
depth, camera poses and intrinsics, and optionally IMU samples and static attachments — for
replay through AR Foundation and for offline processing.

Produced by:

| Producer | Color | Depth |
|---|---|---|
| SensorFlex Recorder, iOS (ARKit) | `rgb.mp4` H.264/HEVC | `lz4_float32` m (default), `zstd_float32`, `hevc_float16`, `h264_uint8_norm` |
| SensorFlex Recorder, Android (ARCore) | `rgb.mp4` H.264/HEVC | `lz4_uint16` mm (default), `hevc_uint8_norm`, `h264_uint8_norm` |
| `data_processing/scannetpp_sfz.py` | `rgb.mp4` re-encoded from the dataset JPEGs | `lz4_uint16` mm (dataset native) |
| `data_processing/hypersim.py` | `rgb.mp4` encoded from the dataset images | `lz4_float32` m |

**Coordinate system invariant:** all poses and vectors are in Unity world space —
left-handed, +Y up, +Z forward, meters. This is a format-level constant; it is not stored
per session.

Earlier versions (1.0 per-frame JPEG archives, 1.1 iOS HEVC archives) are no longer read or
written by any SensorFlex tool.

---

## Archive Structure

```
<session_id>.sfz
└── session/
    ├── session.json      session metadata (required)
    ├── frames.jsonl      one line per camera frame (required)
    ├── rgb.mp4           color video (required when session.json has "rgb")
    ├── depth.bin         depth blocks (block encodings)      ┐ one of, when
    ├── depth.mp4         depth video (video encodings)       ┘ session.json has "depth"
    ├── imu.jsonl         IMU samples (optional)
    └── scene_mesh.ply    attachments (optional, any name listed in "attachments")
```

All file references in `session.json` are relative to `session/`. One archive is one
session; archives are not split into parts.

A session may also be stored unpacked as a directory with the same contents (the player's
FileIo mode reads such a directory directly).

### Zip entry compression

| Entry | Method | Reason |
|---|---|---|
| `session.json`, `frames.jsonl`, `imu.jsonl` | DEFLATE | text |
| `rgb.mp4`, `depth.mp4`, `depth.bin` | STORED | already compressed; STORED entries can be read in place |
| attachments | DEFLATE | e.g. PLY compresses well |

ZIP64 is allowed (and required for archives over 4 GB).

---

## session.json

```json
{
  "sfz_version": "2.0",
  "session_id": "24c603a3e98743719e49825dba32cfb2",
  "start_time_utc": "2026-10-19T15:26:30.000Z",
  "device": { "model": "Google Pixel 6a", "os": "Android OS 17 / API-37", "ar_framework": "ARCore" },
  "actual_fps": 30,
  "frame_count": 352,
  "rgb":   { "width": 1920, "height": 1080, "encoding": "hevc_mp4", "file": "rgb.mp4" },
  "depth": { "width": 160, "height": 90, "encoding": "lz4_uint16", "file": "depth.bin",
             "units": "millimeters", "sensor": "arcore_environment_depth" },
  "imu":   { "file": "imu.jsonl", "sample_rate_hz": 60 },
  "attachments": {
    "scene_mesh": { "file": "scene_mesh.ply", "format": "ply" }
  }
}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `sfz_version` | string | yes | `"2.0"`. Readers should accept any `"2.x"`. |
| `session_id` | string | yes | Unique id; also the archive file name by convention. |
| `start_time_utc` | string | no | ISO-8601 UTC. Empty for converted datasets without a capture time. |
| `device` | object | yes | `model`, `os`, `ar_framework` (`"ARKit"`, `"ARCore"`, `"ScanNet++"`, `"Hypersim"`, …). |
| `actual_fps` | int | yes | Measured average camera frame rate. |
| `frame_count` | int | yes | Number of lines in `frames.jsonl`. |
| `rgb` | object | no | Color stream; omitted when color was not captured. |
| `depth` | object | no | Depth stream; omitted when depth was not captured. |
| `imu` | object | no | IMU stream. |
| `attachments` | object | no | Static assets keyed by name. |

New optional fields may be added within 2.x; readers must ignore fields they do not know.

### rgb

| Field | Description |
|---|---|
| `width`, `height` | Video frame size in pixels. Intrinsics are expressed at this resolution. |
| `encoding` | `"hevc_mp4"` or `"h264_mp4"`. Always 8-bit 4:2:0 (players/hardware decoders cannot be assumed to handle 4:4:4). |
| `file` | `"rgb.mp4"` |

Video frames are matched to frame records through `rgb_frame` (see frames.jsonl), not by
timestamp. Container timestamps are informational.

### depth

| Field | Description |
|---|---|
| `width`, `height` | Depth image size in pixels. |
| `encoding` | See table below. |
| `file` | `"depth.bin"` (block encodings) or `"depth.mp4"` (video encodings). |
| `units` | Unit of a decoded value: `"meters"` or `"millimeters"`. |
| `sensor` | Free-form source label: `"arkit_lidar"`, `"arcore_environment_depth"`, `"scannetpp_lidar"`, `"hypersim_render"`, … |
| `depth_max_meters` | Only for `*_uint8_norm`: the depth that maps to 255. |

| `encoding` | File | Per-frame data | Decoded value |
|---|---|---|---|
| `lz4_float32` | `depth.bin` | one LZ4 raw block (no frame header) of `width×height` float32 LE | value in `units` |
| `lz4_uint16` | `depth.bin` | one LZ4 raw block of `width×height` uint16 LE | value in `units` (normally mm) |
| `zstd_float32` | `depth.bin` | one Zstandard frame of `width×height` float32 LE | value in `units` |
| `hevc_float16` | `depth.mp4` | one video frame, single 16-bit channel holding IEEE float16 | metres |
| `hevc_uint8_norm` / `h264_uint8_norm` | `depth.mp4` | one video frame; luma = depth / `depth_max_meters` × 255 | metres (lossy, coarse) |

Depth images are row-major, first row = top of the image, same orientation as the color
frames. A value of `0` means no depth.

LZ4 blocks must follow the standard LZ4 block format, including its end-of-block rules
(the last 5 bytes are literals), so that reference decoders such as `LZ4_decompress_safe` /
Python `lz4.block.decompress` accept them.

### imu

| Field | Description |
|---|---|
| `file` | `"imu.jsonl"` |
| `sample_rate_hz` | Nominal rate. Use per-sample `ts` for actual timing. |

### attachments

Static assets that are not time series. Each entry: `{"file": "<name>", "format": "<hint>"}`.

| Key | Format | Description |
|---|---|---|
| `scene_mesh` | `"ply"` | Reconstructed static mesh of the environment, in the same Unity world frame as the camera poses. PLY ASCII or binary little-endian; `x y z` required, `nx ny nz` and `red green blue` optional. |

---

## frames.jsonl

One compact JSON object per line, one line per camera frame, in capture order.

```json
{"frame":0,"ts":165324298769666,"pos":[-5.06,3.66,1.95],"rot":[-0.0295,-0.71,0.0687,-0.70],"fx":1521.3,"fy":1511.9,"cx":959.8,"cy":543.8,"rgb_frame":0,"depth_off":0,"depth_sz":22531}
```

| Field | Type | Description |
|---|---|---|
| `frame` | int | Record index, 0-based, contiguous. |
| `ts` | int64 | Capture timestamp, nanoseconds (device clock; only differences are meaningful). |
| `pos` | float[3] | Camera position, Unity world space, metres. |
| `rot` | float[4] | Camera rotation quaternion `[x, y, z, w]`, Unity world space. Camera looks along local +Z, local +Y is up. |
| `fx`, `fy`, `cx`, `cy` | float | Pinhole intrinsics in pixels at the `rgb` resolution. Omitted when unknown. |
| `rgb_frame` | int | Index of this record's frame in `rgb.mp4`, or `-1` if its color frame was dropped. Non-negative values are 0, 1, 2, … in record order. |
| `depth_off`, `depth_sz` | int64, int | Block encodings: byte range of this record's block in `depth.bin`. `depth_sz` = 0 means no depth for this record. |
| `depth_frame` | int | Video encodings: index of this record's frame in `depth.mp4`. |

Readers map video frame *N* to the record whose `rgb_frame == N`; they must not assume
video frame *N* is record *N*.

---

## imu.jsonl

One sample per line, in time order. Body-frame vectors use the Unity convention
(left-handed; device +Z forward).

```json
{"ts":165324298769666,"accel":[0.12,-9.79,0.31],"gyro":[0.001,-0.02,0.004],"gravity":[0.01,-0.999,0.03]}
```

| Field | Type | Description |
|---|---|---|
| `ts` | int64 | Nanoseconds, same clock as `frames.jsonl` `ts`. |
| `accel` | float[3] | Acceleration including gravity, m/s². |
| `gyro` | float[3] | Angular velocity, rad/s. |
| `gravity` | float[3] | Gravity direction, unit vector. Optional. |

---

## Coordinate System

All poses and vectors are in **Unity world space: left-handed, +Y up, +Z forward, metres.**
A decoder assigns `pos` / `rot` directly to `Transform.position` / `Transform.rotation`.

**Converting from ARKit / ScanNet++ (right-handed, +Y up, −Z forward, camera-to-world 4×4):**

1. Flip Z: negate the Z row and Z column (`R' = C·R·C`, `C = diag(1, 1, −1, 1)`; m22 is
   negated twice and stays).
2. Rotate the camera 180° about its local X (post-multiply by `diag(1, −1, −1)`), so the
   optical axis is +Z and image-up is +Y as Unity cameras expect.

Body-frame IMU vectors are Z-flipped the same way. Meshes (e.g. `scene_mesh`) are converted
by the writer, not the reader: negate Z of every vertex and normal and reverse triangle winding
(the Z flip mirrors the mesh).

---

## Decoding examples

**Python** (`data_processing/sfz_reader.py` implements all of this):

```python
import json, zipfile, lz4.block, numpy as np

z = zipfile.ZipFile("session.sfz")
meta = json.loads(z.read("session/session.json"))
frames = [json.loads(l) for l in z.read("session/frames.jsonl").splitlines()]
d = meta["depth"]; blob = z.read("session/" + d["file"])

f = frames[0]
dtype = {"lz4_uint16": "<u2", "lz4_float32": "<f4"}[d["encoding"]]
raw = lz4.block.decompress(blob[f["depth_off"]:f["depth_off"] + f["depth_sz"]],
                           uncompressed_size=d["width"] * d["height"] * np.dtype(dtype).itemsize)
depth = np.frombuffer(raw, dtype).reshape(d["height"], d["width"])
depth_m = depth * (0.001 if d["units"] == "millimeters" else 1.0)
```

**C# / Unity:** see `SfzSessionStore` in `com.sensorflex.player.unity` (video via
`VideoPlayer`, depth blocks via `Lz4BlockDecoder` into an `RFloat` texture in metres).
