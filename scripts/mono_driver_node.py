#!/usr/bin/env python3

"""
Python node for the MonocularMode cpp node.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
Dataset must be configured in EuRoC MAV format
Paths to dataset must be set before building or running this node
Make sure to set path to your workspace in common.hpp

Command line arguments
settings_name: EuRoC, TUM2, KITTI etc; the name of the yaml file containing camera intrinsics and other configurations
image_seq: MH01, V102, etc; the name of the image sequence you want to run
publish_simple_mask: if True, also publish a simple half mask to mask_topic
mask_topic: topic name for mask publishing
mask_top_value: pixel value for the top half of the mask
mask_bottom_value: pixel value for the bottom half of the mask
frame_id: frame_id to stamp into Image headers
"""

import os
import time
from pathlib import Path

import natsort
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from builtin_interfaces.msg import Time as RosTime
from cv_bridge import CvBridge, CvBridgeError


class MonoDriver(Node):
    def __init__(self, node_name: str = "mono_py_node"):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter("settings_name", "EuRoC")
        self.declare_parameter("image_seq", "NULL")

        # New parameters for optional mask publishing
        self.declare_parameter("publish_simple_mask", False)
        self.declare_parameter("mask_topic", "/caustics/mask")
        self.declare_parameter("mask_top_value", 0)
        self.declare_parameter("mask_bottom_value", 255)
        self.declare_parameter("frame_id", "cam0")

        # Parse parameters
        self.settings_name = str(self.get_parameter("settings_name").value)
        self.image_seq = str(self.get_parameter("image_seq").value)

        self.publish_simple_mask = bool(self.get_parameter("publish_simple_mask").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.mask_top_value = int(self.get_parameter("mask_top_value").value)
        self.mask_bottom_value = int(self.get_parameter("mask_bottom_value").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        # Debug prints
        print("-------------- Received parameters --------------------------")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print(f"self.publish_simple_mask: {self.publish_simple_mask}")
        print(f"self.mask_topic: {self.mask_topic}")
        print(f"self.mask_top_value: {self.mask_top_value}")
        print(f"self.mask_bottom_value: {self.mask_bottom_value}")
        print(f"self.frame_id: {self.frame_id}")
        print()

        # Paths
        self.home_dir = str(Path.home()) + "/ros2_ws/src/ros2_orb_slam3"
        self.parent_dir = "TEST_DATASET"
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq
        print(f"self.image_sequence_dir: {self.image_sequence_dir}")
        print()

        # Data lists
        self.imgz_seqz_dir = ""
        self.imgz_seqz = []
        self.time_seqz = []

        # CvBridge
        self.br = CvBridge()

        # Load dataset
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(
            self.image_sequence_dir, "mav0"
        )

        print(f"img_dir: {self.imgz_seqz_dir}")
        print(f"num_images: {len(self.imgz_seqz)}")
        print()

        # ROS topics
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings"
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"

        self.send_config = True

        # Publishers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)

        # Optional mask publisher
        self.publish_mask_msg_ = None
        if self.publish_simple_mask:
            self.publish_mask_msg_ = self.create_publisher(Image, self.mask_topic, 1)

        # Handshake message
        self.exp_config_msg = self.settings_name
        print(f"Configuration to be sent: {self.exp_config_msg}")

        # Subscriber for ack
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )

        # Work variables
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.frame_id_int = 0

        print()
        print("MonoDriver initialized, attempting handshake with CPP node")

    def get_image_dataset_asl(self, exp_dir: str, agent_name: str = "mav0"):
        """
        Returns image file names and timesteps in ascending order from an ASL formatted dataset.
        """
        agent_cam0_fld = exp_dir + "/" + agent_name + "/cam0"
        imgz_file_dir = agent_cam0_fld + "/data/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir), reverse=False)

        time_list = []
        for name in imgz_file_list:
            time_step = name.split(".")[0]
            time_list.append(time_step)

        return imgz_file_dir, imgz_file_list, time_list

    def ack_callback(self, msg: String):
        print(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake_with_cpp_node(self):
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    @staticmethod
    def timestamp_ns_from_filename(imgz_name: str) -> int:
        """
        EuRoC style filename is usually a timestamp in nanoseconds, like 1403636579763555584.png
        """
        base = imgz_name.split(".")[0]
        try:
            return int(base)
        except ValueError:
            return 0

    @staticmethod
    def rostime_from_ns(ts_ns: int) -> RosTime:
        t = RosTime()
        if ts_ns <= 0:
            t.sec = 0
            t.nanosec = 0
            return t
        t.sec = int(ts_ns // 1_000_000_000)
        t.nanosec = int(ts_ns % 1_000_000_000)
        return t

    def build_half_mask(self, height: int, width: int) -> np.ndarray:
        """
        Build a mono8 mask with top half mask_top_value and bottom half mask_bottom_value.
        """
        mask = np.zeros((height, width), dtype=np.uint8)
        mid = height // 2
        mask[:mid, :] = np.uint8(self.mask_top_value)
        mask[mid:, :] = np.uint8(self.mask_bottom_value)
        return mask

    def run_py_node(self, idx: int, imgz_name: str):
        """
        Send RGB image and timestep to the CPP node.
        Optionally publish a half mask with matching header.
        """
        img_look_up_path = self.imgz_seqz_dir + imgz_name

        ts_ns = self.timestamp_ns_from_filename(imgz_name)
        stamp_msg = self.rostime_from_ns(ts_ns)

        timestep_msg = Float64()
        timestep_msg.data = float(ts_ns) if ts_ns > 0 else float(idx)

        bgr = cv2.imread(img_look_up_path, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn(f"Failed to read image: {img_look_up_path}")
            return

        try:
            img_msg = self.br.cv2_to_imgmsg(bgr, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Stamp header
        img_msg.header.stamp = stamp_msg
        img_msg.header.frame_id = self.frame_id

        # Publish image and timestep
        self.publish_timestep_msg_.publish(timestep_msg)
        self.publish_img_msg_.publish(img_msg)

        # Optional mask publish
        if self.publish_mask_msg_ is not None:
            h, w = bgr.shape[0], bgr.shape[1]
            mask_np = self.build_half_mask(h, w)
            try:
                mask_msg = self.br.cv2_to_imgmsg(mask_np, encoding="mono8")
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge error for mask: {e}")
                return

            mask_msg.header.stamp = stamp_msg
            mask_msg.header.frame_id = self.frame_id
            self.publish_mask_msg_.publish(mask_msg)

        self.frame_id_int += 1


def main(args=None):
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")
    rate = n.create_rate(20)

    # Handshake loop
    while n.send_config:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        if not n.send_config:
            break

    print("Handshake complete")

    # Publish loop
    for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n)
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            if n.frame_id_int > n.frame_stop and n.frame_stop != -1:
                print("BREAK!")
                break

        except KeyboardInterrupt:
            break

    cv2.destroyAllWindows()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
