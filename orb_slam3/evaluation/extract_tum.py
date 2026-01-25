import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
import argparse
import os
import glob
import sys

"""
Extract ground truth and odometry data from an MCAP bag file recorded with ORB-SLAM3
RUN: python3 extract_tum.py --mcap <path_to_mcap_file_or_directory>
Example: python3 extract_tum.py --mcap /home/ubuntu/ws_blue/rosbags/my_bag_5/my_bag_5_0.mcap
"""

parser = argparse.ArgumentParser()
parser.add_argument('--mcap', default=None, help='mcap file path')
args = parser.parse_args()

def resolve_bag_path(mcap_arg):
    # If user provided a path
    if mcap_arg:
        p = os.path.expanduser(mcap_arg)
        # If directory, pick first .mcap inside
        if os.path.isdir(p):
            matches = sorted(glob.glob(os.path.join(p, '*.mcap')))
            if not matches:
                sys.exit(f"No .mcap files found in directory: {p}")
            return matches[0]
        # If exact file exists, use it
        if os.path.isfile(p):
            return p
        # Try glob expansion (allow patterns)
        matches = sorted(glob.glob(p))
        if matches:
            return matches[0]
        sys.exit(f"Provided path does not exist or match any files: {mcap_arg}")
    # No argument: search current directory for .mcap
    matches = sorted(glob.glob('*.mcap'))
    if not matches:
        sys.exit("No .mcap file found in current directory. Provide --mcap <path>.")
    return matches[0]

BAG_FILE = resolve_bag_path(args.mcap)
print(f"Using bag file: {BAG_FILE}")

TOPIC_GT = "/model/bluerov2/odometry"
TOPIC_ODO = "/ORB_SLAM3/mono_sim_node/odometry"

OUT_GT = "ground_truth.txt"
OUT_ODO = "odometry.txt"


def write_tum(msg, f):
    ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    f.write(f"{ts:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")


def main():
    rclpy.init()

    # Configure bag reader
    storage_options = StorageOptions(uri=BAG_FILE, storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Lookup topic → type map
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    f_gt = open(OUT_GT, "w")
    f_odo = open(OUT_ODO, "w")

    print("Reading messages...")

    while reader.has_next():
        topic, data, t = reader.read_next()

        # Only decode Odometry messages
        if topic not in type_map:
            continue
        if type_map[topic] != "nav_msgs/msg/Odometry":
            continue

        msg = deserialize_message(data, Odometry)

        if topic == TOPIC_GT:
            write_tum(msg, f_gt)
        elif topic == TOPIC_ODO:
            write_tum(msg, f_odo)

    f_gt.close()
    f_odo.close()

    print("Done.")
    print("Saved:", OUT_GT)
    print("Saved:", OUT_ODO)


if __name__ == "__main__":
    main()
