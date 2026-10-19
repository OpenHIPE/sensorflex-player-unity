# SensorFlex Session Format (SFZ)

Version **2.0**. A self-contained `.sfz` archive (ZIP) holding one recorded session from a
sensor rig — any number of color cameras, depth sensors and IMUs, each with its own pose,
intrinsics and timing — for replay through AR Foundation and for offline processing.

The format makes no assumption about the rig. A handheld AR device (one color camera, depth
from the same viewpoint) and a headset (a world-facing camera plus per-eye depth views) are
both just sets of named sensors.

Produced by:

| Producer | Sensors |
|---|---|
| SensorFlex Recorder, iOS (ARKit) | `color` (`hevc_mp4`/`h264_mp4`), `depth` (`lz4_float32` m by default) |
| SensorFlex Recorder, Android (ARCore) | `color` (`hevc_mp4`/`h264_mp4`), `depth` (`lz4_uint16` mm by default) |
| `data_processing/scannetpp_sfz.py` | `color` (re-encoded dataset JPEGs), `depth` (`lz4_uint16` mm, dataset native), `imu`; `scene_mesh` attachment |
| `data_processing/hypersim.py` | `color`, `depth` (`lz4_float32` m) |

**Coordinate system invariant:** all poses and vectors are in Unity world space —
left-handed, +Y up, +Z forward, meters. This is a format-level constant; it is not stored
per session.

---

## Archive Structure

```
<session_id>.sfz
└── session/
    ├── session.json        session metadata and the sensor list (required)
    ├── frames.jsonl        one line per frame, with a block per sensor (required)
    ├── <sensor data>       one file per camera-like sensor, named in session.json
    │                       (e.g. color.mp4, depth.bin, depth_left.bin, depth_right.bin)
    ├── <imu stream>        one JSONL file per IMU sensor (e.g. imu.jsonl)
    └── <attachments>       static assets (e.g. scene_mesh.ply)
```

All file references in `session.json` are relative to `session/`. One archive is one
session; archives are not split into parts.

A session may also be stored unpacked as a directory with the same contents (the player's
FileIo mode reads such a directory directly).

### Zip entry compression

| Entry | Method | Reason |
|---|---|---|
| `session.json`, `frames.jsonl`, IMU `.jsonl` | DEFLATE | text |
| video (`.mp4`) and depth block (`.bin`) files | STORED | already compressed; STORED entries can be read in place |
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
  "frame_source": "color",
  "sensors": {
    "color": { "type": "color", "file": "color.mp4", "encoding": "hevc_mp4", "width": 1920, "height": 1080 },
    "depth": { "type": "depth", "file": "depth.bin", "encoding": "lz4_uint16", "width": 160, "height": 90,
               "units": "millimeters", "source": "arcore_environment_depth" },
    "imu":   { "type": "imu", "file": "imu.jsonl", "sample_rate_hz": 100 }
  },
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
| `device` | object | yes | `model`, `os`, `ar_framework` (`"ARKit"`, `"ARCore"`, `"AndroidXR"`, `"ScanNet++"`, `"Hypersim"`, …). |
| `actual_fps` | int | yes | Measured average frame rate of `frame_source`. |
| `frame_count` | int | yes | Number of lines in `frames.jsonl`. |
| `frame_source` | string | yes | Key of the sensor whose samples define the frames (normally the main color camera). |
| `sensors` | object | yes | Sensors keyed by a session-unique name (`color`, `depth`, `depth_left`, `color_right`, …). |
| `attachments` | object | no | Static assets keyed by name. |

New optional fields may be added within 2.x; readers must ignore fields they do not know.
Sensor names are free-form; readers find sensors by `type`, not by name.

### Sensor types

Every sensor has `type` and `file`. Further fields depend on the type. A sensor whose data
was not captured at all (e.g. a pose-only recording) has no `file`; its blocks still carry
`ts`, pose and intrinsics, with `index` = -1.

**`color`** — a camera producing images.

| Field | Description |
|---|---|
| `encoding` | `"hevc_mp4"` or `"h264_mp4"`. Always 8-bit 4:2:0 (players/hardware decoders cannot be assumed to handle 4:4:4). |
| `width`, `height` | Frame size in pixels. The sensor's intrinsics are expressed at this resolution. |

**`depth`** — a depth sensor or depth view.

| Field | Description |
|---|---|
| `encoding` | See the table below. |
| `width`, `height` | Depth image size in pixels; intrinsics are expressed at this resolution. |
| `units` | Unit of a decoded value: `"meters"` or `"millimeters"`. |
| `source` | Free-form origin label: `"arkit_lidar"`, `"arcore_environment_depth"`, `"androidxr_depth"`, `"scannetpp_lidar"`, `"hypersim_render"`, … |
| `depth_max_meters` | Only for `*_uint8_norm`: the depth that maps to 255. |

| `encoding` | File | Per-sample data | Decoded value |
|---|---|---|---|
| `lz4_float32` | `.bin` | one LZ4 raw block (no frame header) of `width×height` float32 LE | value in `units` |
| `lz4_uint16` | `.bin` | one LZ4 raw block of `width×height` uint16 LE | value in `units` (normally mm) |
| `zstd_float32` | `.bin` | one Zstandard frame of `width×height` float32 LE | value in `units` |
| `hevc_float16` | `.mp4` | one video frame, single 16-bit channel holding IEEE float16 | metres |
| `hevc_uint8_norm` / `h264_uint8_norm` | `.mp4` | one video frame; luma = depth / `depth_max_meters` × 255 | metres (lossy, coarse) |

Depth images are row-major, first row = top of the image. A value of `0` means no depth.
Depth is *not* assumed to be aligned with any color camera; each depth sample carries its own
pose and intrinsics (see frames.jsonl).

LZ4 blocks must follow the standard LZ4 block format, including its end-of-block rules
(the last 5 bytes are literals), so that reference decoders such as `LZ4_decompress_safe` /
Python `lz4.block.decompress` accept them.

**`imu`** — inertial samples, stored in their own JSONL file at their own rate and not
referenced from frames (see [IMU streams](#imu-streams)).

| Field | Description |
|---|---|
| `sample_rate_hz` | Nominal rate. Use per-sample `ts` for actual timing. |

### attachments

Static assets that are not time series. Each entry: `{"file": "<name>", "format": "<hint>"}`.

| Key | Format | Description |
|---|---|---|
| `scene_mesh` | `"ply"` | Reconstructed static mesh of the environment, in the same Unity world frame as the sensor poses. PLY ASCII or binary little-endian; `x y z` required, `nx ny nz` and `red green blue` optional. |

---

## frames.jsonl

One compact JSON object per line, one line per frame, in time order. A frame is one sample of
`frame_source`; every other camera-like sensor contributes the sample the writer matched to
that frame (normally the nearest in time). Everything needed for a frame is on its line.

```json
{"frame":13,"ts":165324365685760,"color":{"ts":165324365685760,"index":13,"pos":[-5.06,3.66,1.95],"rot":[-0.0295,-0.71,0.0687,-0.70],"fx":1521.3,"fy":1511.9,"cx":959.8,"cy":543.8},"depth":{"ts":165324365685760,"index":13,"pos":[-5.06,3.66,1.95],"rot":[-0.0295,-0.71,0.0687,-0.70],"fx":126.8,"fy":126.0,"cx":80.0,"cy":45.3,"off":293212,"sz":23000}}
```

Line fields:

| Field | Type | Description |
|---|---|---|
| `frame` | int | Frame index, 0-based, contiguous. |
| `ts` | int64 | Timestamp of the `frame_source` sample, nanoseconds. |
| `<sensor name>` | object | One block per camera-like sensor that has a sample for this frame. A sensor with no sample yet (e.g. depth warming up) is omitted. |

Sensor block fields (`color` and `depth` sensors):

| Field | Type | Description |
|---|---|---|
| `ts` | int64 | Capture time of this sensor's sample, nanoseconds. All sensors in a session share one clock; only differences are meaningful. |
| `index` | int | Sample index in the sensor's stream: the frame index in its `.mp4`, or the block number in its `.bin`. Indices start at 0 and increase by 1 per new sample. A sensor slower than `frame_source` repeats the same `index` on consecutive frames. `-1` means the sample was captured (pose and intrinsics are valid) but its data was dropped. |
| `pos` | float[3] | Sensor position, Unity world space, metres. |
| `rot` | float[4] | Sensor rotation quaternion `[x, y, z, w]`, Unity world space. The sensor looks along its local +Z; local +Y is image-up. |
| `fx`, `fy`, `cx`, `cy` | float | Pinhole intrinsics in pixels at the sensor's own `width`×`height`. Omitted when unknown. |
| `off`, `sz` | int64, int | Block depth encodings only: byte range of this sample's block in the sensor's `.bin`. `sz` = 0 means no depth. |

Readers of a video sensor map video frame *N* to the block whose `index == N`; they must not
assume video frame *N* is frame *N*. To visit each depth sample once, skip frames whose depth
`index` equals the previous one.

Converting a symmetric or asymmetric field of view (half-angles `left < 0 < right`,
`down < 0 < up`, radians) to intrinsics at `width`×`height`:
`fx = width / (tan(right) − tan(left))`, `cx = −tan(left) · fx`,
`fy = height / (tan(up) − tan(down))`, `cy = tan(up) · fy`.

### Rig examples

**Handheld AR (phone):** `frame_source` = `color`. `depth` is captured from the color camera's
viewpoint at the same time, so its block repeats the color pose and timestamp, with
intrinsics scaled to the depth resolution, and its `index` advances with the frame.

**Headset (e.g. Android XR):** `frame_source` = `color` (the world-facing camera); `depth_left`
and `depth_right` each carry the pose and intrinsics of their eye view and their own `ts` and
`index`, which may lag or repeat relative to the camera frames.

---

## IMU streams

One JSONL file per `imu` sensor, one sample per line, in time order. Body-frame vectors use
the Unity convention (left-handed; device +Z forward).

```json
{"ts":165324298769666,"accel":[0.12,-9.79,0.31],"gyro":[0.001,-0.02,0.004],"gravity":[0.01,-0.999,0.03]}
```

| Field | Type | Description |
|---|---|---|
| `ts` | int64 | Nanoseconds, same clock as `frames.jsonl`. |
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

name, s = next((k, v) for k, v in meta["sensors"].items() if v["type"] == "depth")
blob = z.read("session/" + s["file"])

d = frames[0][name]                       # this frame's depth sample: pose, intrinsics, block
dtype = {"lz4_uint16": "<u2", "lz4_float32": "<f4"}[s["encoding"]]
raw = lz4.block.decompress(blob[d["off"]:d["off"] + d["sz"]],
                           uncompressed_size=s["width"] * s["height"] * np.dtype(dtype).itemsize)
depth = np.frombuffer(raw, dtype).reshape(s["height"], s["width"])
depth_m = depth * (0.001 if s["units"] == "millimeters" else 1.0)
```

**C# / Unity:** see `SfzSessionStore` in `com.sensorflex.player.unity` (video via
`VideoPlayer`, depth blocks via `Lz4BlockDecoder` into an `RFloat` texture in metres).
