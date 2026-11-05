# MID360 front 180-degree recording

The MID360 connector faces vehicle rear on this vehicle, so sensor +X is
vehicle forward. The filter keeps finite XYZ points with `x > 0` and publishes
`/livox/lidar_front` as the original Livox `CustomMsg` type. Header, frame,
timebase, point timestamps and all point attributes are preserved. Boundary
points at x=0 and invalid points are removed. IMU is not filtered.

For RViz, `/livox/lidar_front_points` provides XYZ `PointCloud2` data when
subscribed. `config/lidar_front180.rviz` sets the topic, `livox_frame`, height
coloring, point size and camera view. Continue recording the `CustomMsg`
topic, which preserves the timing and other Livox attributes.

From the host desktop, run `./start_front180.sh` at the workspace root. It
sets up Docker/X11, builds the filter if necessary, gracefully replaces the
previous Livox display stack, and launches driver, filter and RViz together.
Ctrl+C closes the launch. Recording remains a separate user command.
The launch also registers `livox_display` as a coincident sensor-local TF
child of `livox_frame`; neither is a world pose or a SLAM frame.

Build and test inside the `f1tenth` container:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/f1tenth_ws/install/setup.bash
cmake -S /workspace/f1tenth_ws/scripts/livox_front_cpp \
  -B /workspace/f1tenth_ws/build_front_filter -DCMAKE_BUILD_TYPE=Release
cmake --build /workspace/f1tenth_ws/build_front_filter -j2
ctest --test-dir /workspace/f1tenth_ws/build_front_filter --output-on-failure
```

Run one filter instance with the driver already publishing:

```bash
/workspace/f1tenth_ws/build_front_filter/livox_front_filter \
  --ros-args -p front_yaw_deg:=0.0
```

`front_yaw_deg` is a startup setting measured from +X toward +Y in the input
frame. It selects points without rotating their coordinates. Restart the
filter to change it; use a new bag when changing the selected direction.

Record only `/livox/lidar_front` and `/livox/imu`; recording all topics would
also save the full 360-degree source. Existing full recordings are preserved.
The sensor itself continues scanning 360 degrees, so this reduces recorded
data and downstream work, not sensor network traffic. Point counts depend
on the scene and invalid returns, so output size need not be exactly half.

Current container bag path: `bags/active_recording.path` in the workspace.
Recorder PID and log: `<bag-path>.pid` and `<bag-path>.log`.
Filter PID/log: `bags/livox_front_filter.pid` and
`bags/livox_front_filter_cpp.log`. Send SIGINT to the recorded PID to finish
the bag and write metadata. Do not start a duplicate recorder or filter.

Validation: geometry/metadata unit test passed; 60 live input/output clouds
matched point-for-point with identical metadata, with approximately 10 Hz
LiDAR and 200 Hz IMU observed after the C++ filter was started.
