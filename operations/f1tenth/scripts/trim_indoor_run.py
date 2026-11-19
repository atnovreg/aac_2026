"""Copy a finished indoor drive, keeping five seconds around wheel motion."""
import argparse
import csv
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory


NS = 1_000_000_000
PADDING_NS = 5 * NS
MIN_SPEED = 0.05
MIN_MOTION_NS = 250_000_000
MAX_GAP_NS = 500_000_000


def motion_bounds(samples):
    """Find the outer bounds of sustained movement, ignoring isolated readings."""
    first = last = start = previous = None
    for stamp, speed in samples:
        if not math.isfinite(speed) or speed < MIN_SPEED:
            start = previous = None
            continue
        if start is None or stamp - previous > MAX_GAP_NS:
            start = stamp
        previous = stamp
        if stamp - start >= MIN_MOTION_NS:
            if first is None:
                first = start
            last = stamp
    return (first, last) if first is not None else None


def open_bag(path):
    import rosbag2_py as bag

    reader = bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(path), storage_id="sqlite3"),
                bag.ConverterOptions("", ""))
    return reader


def wheel_speeds(path, config):
    import rosbag2_py as bag
    from rclpy.serialization import deserialize_message
    from vesc_msgs.msg import VescStateStamped
    import yaml

    params = yaml.safe_load(config.read_text())["/vesc/ackermann_to_vesc"]["ros__parameters"]
    gain, offset = params["speed_to_erpm_gain"], params["speed_to_erpm_offset"]
    if not math.isfinite(gain) or gain == 0 or not math.isfinite(offset):
        raise ValueError("Invalid VESC speed conversion")
    reader = open_bag(path)
    reader.set_filter(bag.StorageFilter(topics=["/vesc/sensors/core"]))
    speeds = []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        erpm = deserialize_message(data, VescStateStamped).state.speed
        speeds.append((stamp, abs((erpm - offset) / gain)))
    return speeds


def bag_span(path):
    import yaml

    info = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    start = info["starting_time"]["nanoseconds_since_epoch"]
    return start, start + info["duration"]["nanoseconds"]


def copy_bag(source, destination, start, end):
    import rosbag2_py as bag
    from rclpy.serialization import deserialize_message, serialize_message
    from tf2_msgs.msg import TFMessage

    reader = open_bag(source)
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(destination), storage_id="sqlite3"),
                bag.ConverterOptions("", ""))
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)

    static_transforms = {}
    written = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if stamp < start:
            if topic == "/tf_static":
                for transform in deserialize_message(data, TFMessage).transforms:
                    static_transforms[transform.child_frame_id] = transform
            continue
        if stamp > end:
            break
        if static_transforms:
            writer.write("/tf_static", serialize_message(
                TFMessage(transforms=list(static_transforms.values()))), start)
            static_transforms.clear()
        writer.write(topic, data, stamp)
        written += 1
    writer.close()
    if written == 0:
        raise ValueError(f"No messages in the selected range for {source.name}")


def copy_csv(source, destination, start, end):
    with source.open(newline="") as original, destination.open("x", newline="") as output:
        rows = csv.DictReader(original)
        writer = csv.DictWriter(output, fieldnames=rows.fieldnames)
        writer.writeheader()
        for row in rows:
            if start <= int(row["received_ns"]) <= end:
                writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("manual_config", type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    destination = run / "trim5s"
    if destination.exists():
        print(f"Trimmed copy: {destination}")
        return

    sensors, manual = run / "sensors", run / "ps5"
    sensor_start, sensor_end = bag_span(sensors)
    manual_start, manual_end = bag_span(manual)
    csv_path = run / "mpcc.csv"
    raw_start = min(sensor_start, manual_start)
    raw_end = max(sensor_end, manual_end)
    samples = wheel_speeds(manual, args.manual_config)
    motion = motion_bounds(samples)
    if motion is None:
        print("Trim skipped: no sustained wheel motion in the completed recording.")
        return

    first, last = motion
    start, end = max(raw_start, first - PADDING_NS), min(raw_end, last + PADDING_NS)
    # If a telemetry gap overlaps an interval to be cut, keep that side.
    for previous, following in zip(samples, samples[1:]):
        if following[0] - previous[0] > MAX_GAP_NS:
            if previous[0] < start:
                start = raw_start
            if following[0] > end:
                end = raw_end
    # A bag may start earlier or end later than the wheel stream.
    if samples[0][0] > start:
        start = raw_start
    if samples[-1][0] < end:
        end = raw_end

    with TemporaryDirectory(prefix=".trim-", dir=run) as temporary:
        trimmed = Path(temporary) / "trim5s"
        trimmed.mkdir()
        copy_bag(sensors, trimmed / "sensors", start, end)
        copy_bag(manual, trimmed / "ps5", start, end)
        copy_csv(csv_path, trimmed / "mpcc.csv", start, end)
        summary = dict(start_ns=start, end_ns=end, motion_start_ns=first, motion_end_ns=last,
                       original_start_ns=raw_start, original_end_ns=raw_end,
                       before_motion_s=(first - start) / NS, after_motion_s=(end - last) / NS,
                       duration_s=(end - start) / NS)
        (trimmed / "trim.json").write_text(json.dumps(summary, indent=2) + "\n")
        trimmed.rename(destination)
    print(f"Trimmed copy: {destination} ({summary['duration_s']:.2f} s)")


if __name__ == "__main__":
    main()
