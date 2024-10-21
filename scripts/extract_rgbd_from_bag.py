#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Extract images and depthmaps from rosbag
"""

import time
import os
import pickle
import argparse
import cv2
import numpy as np

import rospy
import rosbag
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class MonkeyPatchTime(rospy.Time):
    def __init__(self, secs=0, nsecs=0):
        super(rospy.Time, self).__init__(secs, nsecs)

    @staticmethod
    def now():
        # initialize with wallclock
        float_secs = time.time()
        secs = int(float_secs)
        nsecs = int((float_secs - secs) * 1000000000)
        return MonkeyPatchTime(secs, nsecs)


rospy.Time = MonkeyPatchTime


def main(args):
    input = args.input
    output = args.output

    image_topic = "/camera/rgb/image_rect_color/compressed"
    depth_topic = "/camera/depth/image_rect"
    sub_image = message_filters.Subscriber(image_topic, CompressedImage)
    sub_depth = message_filters.Subscriber(depth_topic, Image)
    subscriber_list = [sub_image, sub_depth]
    ats_queue_size = 1000  # Max messages in any queue
    ats_slop = 0.1  # Max delay to allow between messages

    # We want a dictionary view of the list for efficiency when dispatching messages
    subscriber_dict = {}
    for subscriber in subscriber_list:
        subscriber_dict[subscriber.topic] = subscriber
    # We want a list with the topic names in the same order to correctly dispatch messages
    topic_names = [subscriber.topic for subscriber in subscriber_list]
    ts = message_filters.ApproximateTimeSynchronizer(
        subscriber_list, queue_size=ats_queue_size, slop=ats_slop, allow_headerless=False
    )

    images = []
    depths = []

    bridge = CvBridge()

    def callback(*msgs):
        # The callback processing the pairs of numbers that arrived at approximately the same time
        for topic_name, msg in zip(topic_names, msgs):
            if topic_name == image_topic:
                cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                images.append(cv_image)
            elif topic_name == depth_topic:
                cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv_depth = np.array(cv_depth, dtype=np.float32)
                depths.append(cv_depth)
            else:
                pass

    ts.registerCallback(callback)

    # Read the input bag
    bag_reader = rosbag.Bag(input, skip_index=True)
    for message_idx, (topic, msg, t) in enumerate(bag_reader.read_messages(topics=topic_names)):
        subscriber = subscriber_dict.get(topic)
        if subscriber:
            subscriber.signalMessage(msg)

    assert (
        len(images) == len(depths)
    ), f"Number of images and depthmaps do not match: {len(images)} != {len(depths)}"
    print(f"Number of images: {len(images)}")

    depth_dir = os.path.join(output, "depth")
    image_dir = os.path.join(output, "image")

    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(image_dir, exist_ok=True)

    for i, (image, depth) in enumerate(zip(images, depths)):
        cv2.imwrite(os.path.join(image_dir, f"{i:05d}.png"), image)
        np.save(os.path.join(depth_dir, f"{i:05d}.npy"), depth)  # depth is a numpy array

    print("Done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images and depthmaps from rosbag")
    parser.add_argument("--input", type=str, required=True, help="Path to the input rosbag")
    parser.add_argument("--output", type=str, default="output", help="Output directory")
    args = parser.parse_args()

    main(args)
