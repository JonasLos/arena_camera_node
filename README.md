# arena_camera_node

ROS 2 driver for Lucid Vision Labs GigE cameras (Arena SDK). Validated against
Triton `TRI032S-C`, firmware `1.69.0.0`.

Publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo`, and exposes the
camera's settings as ROS parameters, most of which can be changed while the
camera is streaming.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `<topic>` | `sensor_msgs/Image` | Path set by the `topic` parameter |
| `<topic namespace>/camera_info` | `sensor_msgs/CameraInfo` | Published beside the image, from `camera_info_url` |

## Services

| Service | Type | Notes |
|---|---|---|
| `<node>/trigger_image` | `std_srvs/Trigger` | Only with `trigger_mode:=true` and `trigger_source:=Software` |

## Runtime tuning

Parameters marked **live** below can be changed on a streaming camera:

```bash
ros2 param set /camera_rear_1/arena_camera_node auto_exposure_target 120.0
```

Everything else is **restart** — the driver rejects the change with a reason
rather than accepting a value that would never take effect:

```
Setting parameter failed: packet_size cannot be changed at runtime; restart arena_camera_node to apply it.
```

Image acquisition runs on its own thread, so parameter and service calls are
served while streaming.

## Parameters

### Device selection and transport

| Parameter | Default | Mode | Description |
|---|---|---|---|
| `ip` | `""` | restart | Camera IP. Takes precedence over `serial`, and is the only selector that narrows discovery to one subnet |
| `serial` | `0` | restart | Camera serial number (integer). Ignored when `ip` is also set |
| `host_iface_ip` | `""` | restart | Host NIC IP to run discovery on |
| `discovery_timeout_ms` | `1000` | restart | Device discovery timeout |
| `packet_size` | `-1` | restart | `GevSCPSPacketSize`. Requires a matching host MTU end to end |
| `packet_delay` | `-1` | restart | `GevSCPD` inter-packet delay |
| `device_link_throughput_limit` | `-1` | restart | Caps device output in bytes/s |
| `stream_buffer_handling_mode` | `""` | restart | `NewestOnly` for lowest latency, `OldestFirstOverwrite` (camera default) to prefer continuity |

### ROS integration

| Parameter | Default | Mode | Description |
|---|---|---|---|
| `topic` | `/<node>/images` | restart | Image topic |
| `frame_id` | `""` | live | TF frame stamped on every message. Falls back to the node namespace, with a warning |
| `camera_info_url` | `""` | live | `file://` URL of the intrinsics YAML. Empty publishes uncalibrated CameraInfo |
| `use_ros_time` | `true` | live | Stamp on arrival. See the warning below before disabling |
| `ptp_enable` | `false` | restart | IEEE-1588 sync. Needs a grandmaster on the network |
| `qos_history` / `qos_history_depth` / `qos_reliability` | `""` / `0` / `""` | restart | Publisher QoS overrides. Depth applies on its own (the publisher is `SensorDataQoS`, already `keep_last`) |

> **Timestamps.** The camera clock counts from power-on and has no relation to
> ROS time. With `use_ros_time:=false` and PTP not locked, image stamps land in
> 1970 and silently break TF lookups, `message_filters` sync, and bag replay.
> Only set it false once `PtpStatus` reports `Master` or `Slave`.

### Exposure and gain

| Parameter | Default | Mode | Description |
|---|---|---|---|
| `auto_exposure` | `true` | live | `ExposureAuto=Continuous` |
| `auto_exposure_min_us` | `100.0` | live | `ExposureAutoLowerLimit` |
| `auto_exposure_max_us` | `60000.0` | live | `ExposureAutoUpperLimit`. See ceiling note below |
| `auto_exposure_target` | `50.0` | live | `TargetBrightness` (integer). Shared by auto exposure **and** auto gain |
| `exposure_time` | `-1.0` | live | Manual exposure µs. Any value ≥ 0 forces `ExposureAuto=Off` and overrides auto exposure |
| `exposure_auto_algorithm` | `""` | live | `Mean` or `Median` metering |
| `exposure_auto_damping` | `-1.0` | live | How fast auto exposure chases scene changes. Lower reacts faster |
| `auto_exposure_aoi_enable` | `false` | live | Restrict metering to a region, e.g. to ignore bright sky |
| `auto_exposure_aoi_width` / `_height` / `_offset_x` / `_offset_y` | `-1` | live | Metering region, in sensor coordinates |
| `gain` | `-1.0` | live | Manual gain dB. Any value ≥ 0 forces `GainAuto=Off` |
| `auto_gain` | `false` | live | `GainAuto=Continuous`. Off by default to match the camera's Default user set |
| `auto_gain_min` / `auto_gain_max` | `-1.0` | live | Auto gain limits in dB (this sensor allows 0–48) |

> **Exposure ceiling.** The camera derives the maximum `ExposureAutoUpperLimit`
> from the current frame rate — at 15 Hz it is about 66.5 ms, at the 33.5 Hz
> power-on default only about 29.7 ms. The driver therefore applies frame rate
> before exposure. A request above the ceiling is clamped to the largest value
> the camera accepts and logged; it is not silently dropped.

### Image format and sensor

| Parameter | Default | Mode | Description |
|---|---|---|---|
| `frame_rate` | `-1.0` | live | Target FPS. Also caps publish rate in software |
| `pixelformat` | `""` | restart | e.g. `bayer_rggb8` |
| `width` / `height` | `0` | restart | ROI size |
| `offset_x` / `offset_y` | `-1` | restart | ROI position. Without these a crop can only be top-left |
| `reverse_x` / `reverse_y` | `false` | restart | Image flip for inverted mounting |
| `adc_bit_depth` | `""` | restart | e.g. `Bits8`, `Bits10`, `Bits12` |
| `balance_white_auto` | `""` | live | `Off`, `Once`, `Continuous` (colour models) |
| `gamma` | `-1.0` | live | Enables `GammaEnable` and sets `Gamma` |
| `black_level` | `-1.0` | live | Sensor black level |

### Triggering

| Parameter | Default | Mode | Description |
|---|---|---|---|
| `trigger_mode` | `false` | restart | Enable triggered acquisition |
| `trigger_source` | `"Software"` | restart | `Software`, `Freerun`, `Line0`, … |
| `trigger_selector` | `"FrameStart"` | restart | Trigger selector |
| `trigger_activation` | `""` | restart | e.g. `RisingEdge`, `FixedRate` |

## Diagnosing settings that do not take effect

Every camera write is logged when it fails, naming the feature and the reason,
and auto exposure and gain values are **read back from the camera** after being
applied. Trust the log line over the requested value:

```
Auto exposure enabled (read back from camera). min_us=9996.640000 max_us=66505.872000 target_brightness=100
ExposureAutoUpperLimit clamped: requested 100000.000000, camera allows [9996.640000, 66505.872000], applied 66505.872000
```

Many GenICam features are conditionally read-only, which is the usual reason a
write is refused:

| Feature | Writable only when |
|---|---|
| `ExposureAutoLowerLimit` / `ExposureAutoUpperLimit` | `ExposureAutoLimitAuto=Off` (the driver sets this first) |
| `ExposureTime` | `ExposureAuto=Off` |
| `AcquisitionFrameRate` | `AcquisitionFrameRateEnable=true` |
| `Gain` | `GainAuto=Off` |
| `AutoExposureAOI*` | `AutoExposureAOIEnable=true` |

To inspect the camera's real feature names, values and access modes:

```bash
cd <ArenaSDK>/ArenaSDK_Linux_x64
export LD_LIBRARY_PATH=$PWD/lib64:$PWD/GenICam/library/lib/Linux64_x64:$LD_LIBRARY_PATH
./precompiledExamples/Cpp_Explore_Nodes
```

## Known limitations

- If the configured camera is not among the discovered devices (unplugged, wrong
  IP, PoE flap, or simply booting slower than this node), device lookup throws
  out of the discovery timer and **the process aborts**. The discovery timer has
  already been cancelled at that point, so there is no retry either way. Run the
  node under a supervisor that restarts it if this matters.
- `qos_history_depth` and the QoS settings are read once at startup.

## Building

```bash
colcon build --packages-select arena_camera_node
```

Requires the Arena SDK (path resolved by `cmake/Findarena_sdk.cmake`) and
`camera_info_manager`.

> This workspace also contains an unused copy of these sources under
> `arena_camera_ros2/ros2_ws/src/arena_camera_node`. That tree carries a
> `COLCON_IGNORE` and is never built — edit `src/arena_camera_node` only.
