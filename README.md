# Xung Multi-Sensor Recorder ROS Package

A command-line ROS recording framework for synchronized Xung intersection data. The recorder launches independent processes for camera streams, ALB lidar point clouds and object tracks, traffic-light states, and weather observations, then creates a shared timestamp index for the completed scene.

## Features

- Records up to six synchronized UHK camera streams.
- Encodes camera frames with NVIDIA hardware acceleration.
- Records ALB edge lidar point clouds, reflectivity, and lidar images.
- Records ALB fusion point clouds and tracked-object trajectories.
- Transforms fused object positions and cuboids into the Xung coordinate system.
- Records traffic-light signal states.
- Records weather station observations.
- Runs sensor recorders in independent CPU-bound processes.
- Reports frame counts and processing times while recording.
- Builds a master timestamp file describing sensor availability.
- Renames completed scenes using the synchronized recording start time.

## Repository Structure

```text
.
|-- Dockerfile
|-- docker-compose.yaml
|-- entrypoint.sh
|-- entrypoint_dev.sh
|-- requirements.txt
`-- catkin_ws/
    `-- src/xung_recorder_pkg/
        |-- config/config.json
        |-- msg/
        |   |-- AugmentedCloud.msg
        |   |-- ObjectData.msg
        |   `-- TrackedObjects.msg
        |-- src/
        |   |-- recorder.py       # Main process orchestration
        |   |-- camera.py         # GPU camera stream recorder
        |   |-- decoder.py        # Video decoding helper
        |   |-- alb_edge.py       # Edge lidar recorder
        |   |-- alb_fusion.py     # Fusion lidar and trajectory recorder
        |   |-- lsa.py            # Traffic-light recorder
        |   |-- weather.py        # Weather recorder
        |   `-- utils.py          # Lidar destaggering and CLI helpers
        |-- CMakeLists.txt
        `-- package.xml
```

## Requirements

### Hardware

- NVIDIA GPU with NVENC support for camera recording
- Sufficient CPU cores for one process per configured sensor
- High-throughput storage for simultaneous camera and lidar data
- Network access to all ROS sensor publishers

### Software

- Linux
- Docker Engine and Docker Compose, or a compatible native environment
- NVIDIA Container Toolkit
- CUDA 11.5-compatible NVIDIA driver
- ROS Noetic and catkin
- FFmpeg shared libraries
- NVIDIA Video Processing Framework / `PyNvCodec`
- Python 3

The Python packages listed in `requirements.txt` include NumPy, OpenCV, Ouster SDK, psutil, and supporting data-processing libraries.

## Container Build Status

The current Dockerfile is **not self-contained for a clean public build**. It uses:

```text
nvcr.io/nvidia/cuda:11.5.0-devel-ubuntu20.04
```

but it does not install:

- ROS Noetic
- catkin tools
- FFmpeg at the paths expected by the entrypoint
- NVIDIA Video Processing Framework / `PyNvCodec`

The entrypoint expects these external paths:

```text
/opt/ros/noetic
/workspace/Git/VideoProcessingFramework/install/bin
/workspace/Git/FFmpeg/build_x64_release_shared/lib
```

Provide those dependencies in the image or use the original prebuilt image before running the recorder. A plain `docker compose build` from this checkout may complete the base Python installation but will not produce a functional recorder environment without the missing native components.

## Docker Setup

The Compose file contains machine-specific host paths and CPU IDs. Update them for the deployment host:

```yaml
volumes:
  - /absolute/path/to/recordings:/workspace/data
  - /absolute/path/to/xung_recorder_pkg-main:/workspace/repos
```

The service uses host networking for ROS communication:

```yaml
network_mode: host
```

The default CPU set is:

```yaml
cpuset: "40,41,42,43,44,45,46,47,48,49,50,51,52"
```

Replace it with valid host CPU IDs and keep it consistent with `cpu_id` in `config/config.json`.

After preparing a complete image, start the development container:

```bash
docker compose up -d
```

Open a shell:

```bash
docker compose exec xung_recorder_pkg_dev bash
```

Stop or remove the container:

```bash
docker compose stop
docker compose down
```

GPU access is requested through the Compose device reservation and NVIDIA driver capabilities.

## Python Dependencies

Install the Python packages with:

```bash
pip install -r requirements.txt
```

This does not install ROS, FFmpeg, CUDA, or `PyNvCodec`. Those native dependencies must be installed separately and exposed through `PYTHONPATH` and `LD_LIBRARY_PATH`.

## Build the ROS Workspace

In a complete ROS environment:

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/repos/catkin_ws
catkin build
source devel/setup.bash
```

The package generates three custom message types:

- `AugmentedCloud`
- `ObjectData`
- `TrackedObjects`

## Configuration

Recording targets are configured in:

```text
catkin_ws/src/xung_recorder_pkg/config/config.json
```

Default configuration:

```json
{
  "cpu_id": 40,
  "albs": ["edge_m1", "edge_m2", "edge_m3", "fusion"],
  "cams": ["uhk1", "uhk2", "uhk3", "uhk4", "uhk5", "uhk6"],
  "context": ["weather", "lsa"],
  "recording_dir": "/workspace/data"
}
```

- `cpu_id`: first CPU assigned to recorder processes
- `albs`: ALB lidar sources to record
- `cams`: camera sources to record
- `context`: optional weather and traffic-light sources
- `recording_dir`: destination root for completed scenes

One additional CPU is assigned for each camera and ALB process. Camera processes also rotate through configured GPU IDs.

## Required ROS Topics

### Timestamp

```text
xung_timestamp_node                  std_msgs/UInt64
```

This synchronized timestamp drives scene naming and sensor metadata.

### Cameras

The camera recorder expects UHK image topics associated with the configured names. Camera messages use:

```text
sensor_msgs/Image
```

### ALB Edge Sensors

For an edge source such as `edge_m1`:

```text
alb_edge_m1_augmented_cloud          xung_recorder_pkg/AugmentedCloud
```

### ALB Fusion

```text
alb_fusion_augmented_cloud           xung_recorder_pkg/AugmentedCloud
alb_fusion_tracked_objects           xung_recorder_pkg/TrackedObjects
```

### Context Sensors

```text
xung_weather_data                    std_msgs/Float32MultiArray
```

The LSA recorder subscribes to the configured traffic-light state source used by the Xung stack.

## Start a Recording

Source ROS and the catkin workspace, then run:

```bash
cd /workspace/repos
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
rosrun xung_recorder_pkg recorder.py
```

The production entrypoint adds this alias:

```bash
alias rec='rosrun xung_recorder_pkg recorder.py'
```

Recording is interactive:

1. Start the recorder and wait for all sensor processes to initialize.
2. Press `Enter` to begin recording.
3. Press `Enter` again to stop.
4. Wait for every recorder process to flush data and exit.
5. The temporary `current_recording` directory is renamed using the synchronized start time.

Do not terminate the process while recorders are flushing metadata and closing video encoders.

## Output Structure

A completed recording contains sensor-specific data below a timestamped scene directory. Typical output resembles:

```text
scene_timestamp/
|-- camera/
|   |-- raw/
|   |-- resized/
|   `-- metadata JSON files
|-- alb/
|   |-- edge_m1/
|   |   |-- augmented_cloud/
|   |   |-- reflectivity/
|   |   `-- lidar_image/
|   |-- edge_m2/
|   |-- edge_m3/
|   `-- fusion/
|       |-- augmented_cloud/
|       `-- trajectory metadata
|-- meta/
|   |-- weather/
|   `-- lsa/
`-- master timestamp data
```

Exact files depend on the enabled sensors.

## Camera Recording

Camera frames are encoded using NVIDIA Video Processing Framework. The camera recorder tracks:

- Frame count
- Processing time
- Image timestamps
- Stream metadata

Ensure the selected GPU supports the requested encoder format and that sufficient encoder sessions are available for all configured cameras.

## Lidar Recording

### Edge Lidar

The edge recorder destaggers Ouster-style point data, stores XYZ and reflectivity arrays, and creates human-readable lidar and reflectivity BMP images.

### Fusion Lidar

The fusion recorder stores point clouds with reflectivity and object IDs. It also records object classes, poses, cuboids, box dimensions, velocity, and transformed Xung coordinates.

## Context Recording

### Weather

Weather records include pressure, temperature, humidity, wind, precipitation, visibility, and timestamp data.

### Traffic Lights

The LSA recorder stores synchronized traffic-light state observations for later integration with trajectories and scene metadata.

## Master Timestamp Index

After all sensor processes stop, `create_master_ts_file` combines available camera, ALB, weather, and traffic-light timestamps. Sensor samples are matched to exact or nearest master timestamps, producing a synchronized availability index for downstream processing.

## License

his project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details
