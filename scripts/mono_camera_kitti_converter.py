#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: BSD-3-Clause
"""Convert a single-camera ROS 2 MCAP bag to a KITTI-like dataset.

This converter is intended for bags that contain one camera stream, such as
Cyclair nav1 HD bags. Images are written directly to KITTI `image_/` while the
bag is read, so the conversion does not keep full image sequences in memory.
"""

import argparse
import os
import re
from collections import deque
from typing import Dict, Iterable, List, Optional, Set, TextIO, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from tf_transformations import quaternion_from_matrix, quaternion_matrix


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def message_stamp_or_bag_time(message, bag_time_ns: int) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        return stamp_to_sec(stamp)
    return bag_time_ns * 1e-9


def write_png(path: str, image) -> None:
    ok = cv2.imwrite(path, image, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError(f"[ERROR] Failed to write PNG: {path}")


def find_closest_timestamp(target: float, stamps: Iterable[float]) -> Optional[int]:
    best_i = None
    best_dt = None
    for i, stamp in enumerate(stamps):
        dt = abs(stamp - target)
        if best_dt is None or dt < best_dt:
            best_i = i
            best_dt = dt
    return best_i


def sanitize_topic_name(topic: str) -> str:
    name = topic.strip("/") or "root"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name)


def get_topic_types(reader: SequentialReader) -> Dict[str, str]:
    return {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }


def normalize_frame_id(frame_id: str) -> str:
    return frame_id.strip().lstrip("/")


def compact_frame_id(frame_id: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", normalize_frame_id(frame_id).lower())


def camera_frame_candidate_score(
    frame_id: str,
    detected_camera_frame: Optional[str],
    image_topic: str,
) -> int:
    frame = normalize_frame_id(frame_id).lower()
    compact_frame = compact_frame_id(frame_id)
    compact_detected = compact_frame_id(detected_camera_frame or "")
    compact_topic = compact_frame_id(image_topic)

    score = 0
    if compact_detected and compact_frame == compact_detected:
        score += 100
    if compact_detected and (
        compact_frame in compact_detected or compact_detected in compact_frame
    ):
        score += 40
    if "cam" in frame or "camera" in frame:
        score += 10
    if "nav" in frame:
        score += 25
    if "optical" in frame:
        score += 10
    if "tools" in frame:
        score -= 20
    if "nav1" in compact_topic and "nav1" in compact_frame:
        score += 40
    if "nav1" in compact_topic and "nav" in compact_frame and "1" in compact_frame:
        score += 40

    return score


def matrix_from_transform(transform) -> np.ndarray:
    t = transform.translation
    q = transform.rotation
    matrix = quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[0, 3] = t.x
    matrix[1, 3] = t.y
    matrix[2, 3] = t.z
    return matrix


def inverse_transform(matrix: np.ndarray) -> np.ndarray:
    return np.linalg.inv(matrix)


def matrix_translation_quaternion(matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    translation = matrix[:3, 3]
    quaternion = quaternion_from_matrix(matrix)
    return translation, quaternion


class TfTransformCollector:
    def __init__(self) -> None:
        self.transforms: Dict[Tuple[str, str], Tuple[np.ndarray, str, float]] = {}

    def add_message(self, topic: str, msg, bag_time_ns: int) -> None:
        bag_time = bag_time_ns * 1e-9
        for transform in msg.transforms:
            parent = normalize_frame_id(transform.header.frame_id)
            child = normalize_frame_id(transform.child_frame_id)
            if not parent or not child:
                continue
            self.transforms[(parent, child)] = (
                matrix_from_transform(transform.transform),
                topic,
                bag_time,
            )

    def find_transform(
        self,
        target_frame: str,
        source_frame: str,
    ) -> Tuple[Optional[np.ndarray], List[str]]:
        target_frame = normalize_frame_id(target_frame)
        source_frame = normalize_frame_id(source_frame)
        if not target_frame or not source_frame:
            return None, []
        if target_frame == source_frame:
            return np.eye(4), [source_frame]

        adjacency: Dict[str, List[Tuple[str, np.ndarray, str]]] = {}
        for (parent, child), (parent_t_child, topic, bag_time) in self.transforms.items():
            adjacency.setdefault(child, []).append(
                (
                    parent,
                    parent_t_child,
                    f"{child} -> {parent} ({topic}, t={bag_time:.9f})",
                )
            )
            adjacency.setdefault(parent, []).append(
                (
                    child,
                    inverse_transform(parent_t_child),
                    f"{parent} -> {child} ({topic}, t={bag_time:.9f})",
                )
            )

        queue = deque([(source_frame, np.eye(4), [source_frame])])
        visited = {source_frame}
        while queue:
            frame, target_t_frame, path = queue.popleft()
            if frame == target_frame:
                return target_t_frame, path
            for next_frame, next_t_frame, edge_label in adjacency.get(frame, []):
                if next_frame in visited:
                    continue
                visited.add(next_frame)
                queue.append((next_frame, next_t_frame @ target_t_frame, path + [edge_label]))

        return None, []

    def frames(self) -> List[str]:
        frame_names = set()
        for parent, child in self.transforms:
            frame_names.add(parent)
            frame_names.add(child)
        return sorted(frame_names)

    def find_best_camera_frame(
        self,
        robot_frame: str,
        detected_camera_frame: Optional[str],
        image_topic: str,
    ) -> Tuple[Optional[str], Optional[np.ndarray], List[str]]:
        candidates = []
        for frame in self.frames():
            score = camera_frame_candidate_score(frame, detected_camera_frame, image_topic)
            if score <= 0:
                continue
            robot_t_camera, chain = self.find_transform(robot_frame, frame)
            if robot_t_camera is None:
                continue
            candidates.append((score, -len(chain), frame, robot_t_camera, chain))

        if not candidates:
            return None, None, []

        candidates.sort(reverse=True, key=lambda item: (item[0], item[1], item[2]))
        _, _, frame, robot_t_camera, chain = candidates[0]
        return frame, robot_t_camera, chain


def write_matrix(
    handle: TextIO,
    name: str,
    matrix: np.ndarray,
    indent: str = "",
) -> None:
    handle.write(f"{indent}{name}:\n")
    for row in matrix:
        handle.write(
            indent + "  " + " ".join(f"{value:.9f}" for value in row) + "\n"
        )


def write_nav_camera_transform(
    path: str,
    tf_collector: TfTransformCollector,
    camera_frame: Optional[str],
    robot_frame: str,
    image_topic: str,
) -> Tuple[bool, Optional[str]]:
    detected_camera_frame = normalize_frame_id(camera_frame or "") or None
    resolved_camera_frame = detected_camera_frame
    robot_t_camera = None
    chain: List[str] = []

    if resolved_camera_frame:
        robot_t_camera, chain = tf_collector.find_transform(
            robot_frame,
            resolved_camera_frame,
        )

    if robot_t_camera is None:
        resolved_camera_frame, robot_t_camera, chain = tf_collector.find_best_camera_frame(
            robot_frame=robot_frame,
            detected_camera_frame=detected_camera_frame,
            image_topic=image_topic,
        )

    with open(path, "w") as f:
        f.write("# Transform between nav camera and robot center\n")
        f.write(f"robot_frame: {normalize_frame_id(robot_frame)}\n")
        f.write(f"detected_camera_frame: {detected_camera_frame or ''}\n")
        f.write(f"resolved_camera_frame: {resolved_camera_frame or ''}\n")
        f.write("source_topics: /tf_static, /tf\n\n")

        if not detected_camera_frame and not resolved_camera_frame:
            f.write("status: not_found\n")
            f.write("reason: no camera frame_id found in Image or CameraInfo header\n")
            f.write("available_tf_frames:\n")
            for frame in tf_collector.frames():
                f.write(f"  - {frame}\n")
            return False, None

        if robot_t_camera is None or not resolved_camera_frame:
            f.write("status: not_found\n")
            f.write("reason: no TF chain found between camera_frame and robot_frame\n")
            f.write("available_tf_frames:\n")
            for frame in tf_collector.frames():
                f.write(f"  - {frame}\n")
            return False, detected_camera_frame

        camera_t_robot = inverse_transform(robot_t_camera)
        t_robot_camera, q_robot_camera = matrix_translation_quaternion(robot_t_camera)
        t_camera_robot, q_camera_robot = matrix_translation_quaternion(camera_t_robot)
        robot_label = normalize_frame_id(robot_frame).replace("/", "_")

        f.write("status: found\n")
        f.write("convention: T_target_source transforms coordinates from source frame to target frame\n\n")
        f.write("tf_chain_camera_to_robot:\n")
        for item in chain:
            f.write(f"  - {item}\n")
        f.write("\n")

        f.write(f"T_{robot_label}_camera:\n")
        f.write("  source_frame: " + normalize_frame_id(resolved_camera_frame) + "\n")
        f.write("  target_frame: " + normalize_frame_id(robot_frame) + "\n")
        f.write(
            "  translation_xyz: "
            + " ".join(f"{value:.9f}" for value in t_robot_camera)
            + "\n"
        )
        f.write(
            "  quaternion_xyzw: "
            + " ".join(f"{value:.9f}" for value in q_robot_camera)
            + "\n"
        )
        write_matrix(f, "matrix_4x4", robot_t_camera, indent="  ")
        f.write("\n")

        f.write(f"T_camera_{robot_label}:\n")
        f.write("  source_frame: " + normalize_frame_id(robot_frame) + "\n")
        f.write("  target_frame: " + normalize_frame_id(resolved_camera_frame) + "\n")
        f.write(
            "  translation_xyz: "
            + " ".join(f"{value:.9f}" for value in t_camera_robot)
            + "\n"
        )
        f.write(
            "  quaternion_xyzw: "
            + " ".join(f"{value:.9f}" for value in q_camera_robot)
            + "\n"
        )
        write_matrix(f, "matrix_4x4", camera_t_robot, indent="  ")
        return True, resolved_camera_frame


class TopicTextExporter:
    def __init__(
        self,
        out_dir: str,
        topic_types: Dict[str, str],
        excluded_topics: Set[str],
        enabled: bool,
    ) -> None:
        self.out_dir = out_dir
        self.topic_types = topic_types
        self.excluded_topics = excluded_topics
        self.enabled = enabled
        self.handles: Dict[str, TextIO] = {}
        self.message_types: Dict[str, Optional[type]] = {}
        self.counts: Dict[str, int] = {}
        if self.enabled:
            ensure_dir(self.out_dir)

    def close(self) -> None:
        for handle in self.handles.values():
            handle.close()
        self.handles.clear()

    def _handle_for(self, topic: str) -> TextIO:
        handle = self.handles.get(topic)
        if handle is not None:
            return handle

        path = os.path.join(self.out_dir, f"{sanitize_topic_name(topic)}.txt")
        handle = open(path, "w")
        handle.write(f"# topic: {topic}\n")
        handle.write(f"# type: {self.topic_types.get(topic, 'unknown')}\n")
        handle.write("# format: one record per message, separated by '---'\n\n")
        self.handles[topic] = handle
        return handle

    def _message_type_for(self, topic: str):
        if topic in self.message_types:
            return self.message_types[topic]

        type_name = self.topic_types.get(topic)
        if not type_name:
            self.message_types[topic] = None
            return None
        try:
            msg_type = get_message(type_name)
        except Exception:
            msg_type = None
        self.message_types[topic] = msg_type
        return msg_type

    def write(self, topic: str, data: bytes, bag_time_ns: int) -> None:
        if not self.enabled or topic in self.excluded_topics:
            return

        handle = self._handle_for(topic)
        msg_type = self._message_type_for(topic)
        count = self.counts.get(topic, 0)
        self.counts[topic] = count + 1

        handle.write("---\n")
        handle.write(f"index: {count}\n")
        handle.write(f"bag_time: {bag_time_ns * 1e-9:.9f}\n")
        handle.write(f"type: {self.topic_types.get(topic, 'unknown')}\n")

        if msg_type is None:
            handle.write("deserialized: false\n")
            handle.write(f"raw_bytes: {len(data)}\n")
            handle.write(
                "note: message type unavailable in this ROS environment\n\n"
            )
            return

        try:
            msg = deserialize_message(data, msg_type)
        except Exception as exc:
            handle.write("deserialized: false\n")
            handle.write(f"raw_bytes: {len(data)}\n")
            handle.write(f"error: {type(exc).__name__}: {exc}\n\n")
            return

        handle.write("deserialized: true\n")
        handle.write(f"stamp: {message_stamp_or_bag_time(msg, bag_time_ns):.9f}\n")
        handle.write("message:\n")
        for line in str(msg).splitlines():
            handle.write(f"  {line}\n")
        handle.write("\n")


def write_readme(
    seq_dir: str,
    bag_dir: str,
    seq: str,
    image_topic: str,
    camera_info_topic: str,
    odom_topic: str,
    robot_frame: str,
    camera_frame: Optional[str],
    nav_camera_transform_found: bool,
    exported_topics: Dict[str, int],
) -> None:
    path = os.path.join(seq_dir, "README.md")
    with open(path, "w") as f:
        f.write(f"# KITTI Mono-Camera Conversion: {seq}\n\n")
        f.write("## Source\n")
        f.write(f"- ROS 2 bag: `{os.path.abspath(bag_dir)}`\n")
        f.write("- Storage: MCAP\n\n")
        f.write("## KITTI Layout\n")
        f.write("- `image_/`: PNG images from the single camera stream.\n")
        f.write("- `times.txt`: one timestamp per `image_` frame.\n")
        f.write("- `poses.txt`: nearest `/odom/sbg` pose aligned to each frame.\n")
        f.write("- `calib.txt`: `P2` projection matrix from camera info.\n")
        f.write("- `nav_camera_to_imu_link.txt`: TF transform between nav camera and robot center.\n")
        f.write("- `info.txt`: conversion metadata.\n")
        f.write("- `topics/`: optional text export of non-KITTI topics.\n\n")
        f.write("## Topics Used For KITTI\n")
        f.write(f"- Image: `{image_topic}` -> `image_/`\n")
        f.write(f"- CameraInfo: `{camera_info_topic}` -> `calib.txt` (`P2`)\n")
        f.write(f"- Odometry: `{odom_topic}` -> `poses.txt`\n\n")
        f.write("## Nav Camera Transform\n")
        f.write(f"- Robot center frame: `{normalize_frame_id(robot_frame)}`\n")
        f.write(f"- Camera frame: `{normalize_frame_id(camera_frame or '') or 'not detected'}`\n")
        status = "found" if nav_camera_transform_found else "not found"
        f.write(f"- Status: `{status}` -> `nav_camera_to_imu_link.txt`\n\n")
        f.write("## Exported Unused Topics\n")
        if exported_topics:
            for topic, count in sorted(exported_topics.items()):
                filename = sanitize_topic_name(topic) + ".txt"
                f.write(f"- `{topic}` -> `topics/{filename}` ({count} messages)\n")
        else:
            f.write("- None.\n")


def convert_mono_camera_bag(
    bag_dir: str,
    out_root: str,
    seq: str,
    image_topic: str,
    camera_info_topic: str,
    odom_topic: str,
    robot_frame: str,
    camera_frame: Optional[str],
    export_unused_topics: bool,
) -> None:
    seq_dir = os.path.join(out_root, "sequences", seq)
    if os.path.exists(seq_dir):
        raise FileExistsError(
            f"[ERROR] Sequence folder already exists: {seq_dir}\n"
            "Delete or rename it before running again."
        )

    image_dir = os.path.join(seq_dir, "image_")
    topics_dir = os.path.join(seq_dir, "topics")
    ensure_dir(image_dir)

    info_path = os.path.join(seq_dir, "info.txt")
    with open(info_path, "w") as f:
        f.write("KITTI Mono-Camera Conversion Metadata\n")
        f.write("=====================================\n")
        f.write(f"Source ROS2 bag : {os.path.abspath(bag_dir)}\n")
        f.write(f"Output sequence : {seq}\n")
        f.write(f"Image topic     : {image_topic}\n")
        f.write(f"CameraInfo topic: {camera_info_topic}\n")
        f.write(f"Odometry topic  : {odom_topic}\n")
        f.write(f"Robot frame     : {normalize_frame_id(robot_frame)}\n")
        f.write(f"Camera frame    : {normalize_frame_id(camera_frame or '') or 'auto'}\n")
        f.write(f"Export topics   : {export_unused_topics}\n")

    storage_options = StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = get_topic_types(reader)

    bridge = CvBridge()
    image_msg_type = get_message("sensor_msgs/msg/Image")
    info_msg_type = get_message("sensor_msgs/msg/CameraInfo")
    odom_msg_type = get_message("nav_msgs/msg/Odometry")
    tf_msg_type = get_message("tf2_msgs/msg/TFMessage")

    excluded_topics = {image_topic, camera_info_topic, odom_topic}
    exporter = TopicTextExporter(
        topics_dir,
        topic_types,
        excluded_topics,
        enabled=export_unused_topics,
    )

    frame_times = []
    odom_times = []
    odom_poses = []
    camera_p = None
    camera_frame_id = normalize_frame_id(camera_frame or "") or None
    tf_collector = TfTransformCollector()
    image_count = 0

    print(f"[INFO] Bag directory : {bag_dir}")
    print(f"[INFO] Output KITTI  : {seq_dir}")
    print("[INFO] Reading bag messages...")
    try:
        while reader.has_next():
            topic, data, bag_time_ns = reader.read_next()

            if topic == image_topic:
                msg = deserialize_message(data, image_msg_type)
                if camera_frame_id is None:
                    camera_frame_id = normalize_frame_id(msg.header.frame_id) or None
                stamp = stamp_to_sec(msg.header.stamp)
                image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                write_png(os.path.join(image_dir, f"{image_count:06d}.png"), image)
                frame_times.append(stamp)
                image_count += 1
                continue

            if topic == camera_info_topic and camera_p is None:
                msg = deserialize_message(data, info_msg_type)
                if camera_frame_id is None:
                    camera_frame_id = normalize_frame_id(msg.header.frame_id) or None
                camera_p = list(msg.p) if any(msg.p) else None
                print("[INFO] Captured camera P")
                continue

            if topic == odom_topic:
                msg = deserialize_message(data, odom_msg_type)
                stamp = stamp_to_sec(msg.header.stamp)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                transform = quaternion_matrix([q.x, q.y, q.z, q.w])
                transform[0, 3] = p.x
                transform[1, 3] = p.y
                transform[2, 3] = p.z
                odom_times.append(stamp)
                odom_poses.append(transform[:3, :].reshape(-1))
                continue

            if topic in ("/tf_static", "/tf"):
                msg = deserialize_message(data, tf_msg_type)
                tf_collector.add_message(topic, msg, bag_time_ns)
                exporter.write(topic, data, bag_time_ns)
                continue

            exporter.write(topic, data, bag_time_ns)
    finally:
        exporter.close()

    if not frame_times:
        raise RuntimeError(f"[ERROR] No images extracted from topic: {image_topic}")
    if camera_p is None:
        raise RuntimeError(
            f"[ERROR] Missing valid CameraInfo P on topic: {camera_info_topic}"
        )

    times_path = os.path.join(seq_dir, "times.txt")
    with open(times_path, "w") as f:
        for stamp in frame_times:
            f.write(f"{stamp:.9f}\n")

    poses_path = os.path.join(seq_dir, "poses.txt")
    with open(poses_path, "w") as f:
        for stamp in frame_times:
            idx = find_closest_timestamp(stamp, odom_times)
            pose = np.eye(3, 4).reshape(-1) if idx is None else odom_poses[idx]
            f.write(" ".join(f"{value:.9f}" for value in pose) + "\n")

    calib_path = os.path.join(seq_dir, "calib.txt")
    with open(calib_path, "w") as f:
        f.write("P2: " + " ".join(f"{value:.9f}" for value in camera_p) + "\n")

    nav_camera_transform_path = os.path.join(seq_dir, "nav_camera_to_imu_link.txt")
    nav_camera_transform_found, resolved_camera_frame = write_nav_camera_transform(
        path=nav_camera_transform_path,
        tf_collector=tf_collector,
        camera_frame=camera_frame_id,
        robot_frame=robot_frame,
        image_topic=image_topic,
    )

    write_readme(
        seq_dir=seq_dir,
        bag_dir=bag_dir,
        seq=seq,
        image_topic=image_topic,
        camera_info_topic=camera_info_topic,
        odom_topic=odom_topic,
        robot_frame=robot_frame,
        camera_frame=resolved_camera_frame or camera_frame_id,
        nav_camera_transform_found=nav_camera_transform_found,
        exported_topics=exporter.counts,
    )

    print("\nConversion complete.")
    print(f"  KITTI-like data in: {seq_dir}")
    print(f"  image_ frames      : {len(frame_times)}")
    print(f"  odom poses read    : {len(odom_times)}")
    print(f"  nav camera TF      : {'found' if nav_camera_transform_found else 'not found'}")
    print(f"  exported topics    : {len(exporter.counts)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a single-camera ROS 2 MCAP bag to KITTI-like format."
    )
    parser.add_argument(
        "--bag",
        required=True,
        help="Path to ROS 2 bag directory containing metadata.yaml and .mcap files.",
    )
    parser.add_argument(
        "--out",
        required=True,
        help="Output root directory. The sequence is written under sequences/<seq>/.",
    )
    parser.add_argument(
        "--seq",
        required=True,
        help="Sequence id under the output sequences/ directory.",
    )
    parser.add_argument(
        "--image-topic",
        default="/cam_nav1/hd/image_color_rect",
        help="Single camera image topic written to image_/.",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/cam_nav1/hd/camera_info",
        help="CameraInfo topic used to write P2 in calib.txt.",
    )
    parser.add_argument(
        "--odom-topic",
        default="/odom/sbg",
        help="nav_msgs/Odometry topic used to generate poses.txt.",
    )
    parser.add_argument(
        "--robot-frame",
        default="imu_link",
        help="Robot center frame used for nav_camera_to_imu_link.txt.",
    )
    parser.add_argument(
        "--camera-frame",
        default=None,
        help=(
            "Camera frame used for nav_camera_to_imu_link.txt. "
            "Defaults to the Image/CameraInfo header.frame_id."
        ),
    )
    parser.add_argument(
        "--no-export-unused-topics",
        action="store_true",
        help="Disable text export of topics not used for KITTI images/calib/poses.",
    )

    args = parser.parse_args()
    convert_mono_camera_bag(
        bag_dir=args.bag,
        out_root=args.out,
        seq=args.seq,
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        odom_topic=args.odom_topic,
        robot_frame=args.robot_frame,
        camera_frame=args.camera_frame,
        export_unused_topics=not args.no_export_unused_topics,
    )


if __name__ == "__main__":
    main()
