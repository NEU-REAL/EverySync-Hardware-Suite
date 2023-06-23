#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Taken from http://wiki.ros.org/rosbag/Cookbook
# Test with python3.6
# pip3 install tqdm before using tqdm
import os
import rosbag
import rospy
import logging
from tqdm import tqdm
import argparse


def parse_args():
    parser = argparse.ArgumentParser(description='Cook a bag file.')
    #parser.add_argument('--inputbag_path', type=str, default=None, help='input bag file path')
    parser.add_argument('--i', type=str, default=None, help='input bag file path')
    parser.add_argument('--outputbag_path', type=str, default=None, help='output bag file path')
    parser.add_argument('--start_time', type=float, default=None, help='start time in secs, eg. 1671521845.295684905')
    parser.add_argument('--end_time', type=float, default=None, help='end time in secs, eg. 1671521951.471916482')

    return parser.parse_args()


def main():
    args = parse_args()
    print(args)
    i = args.i
    outputbag_path = args.outputbag_path
    if outputbag_path is None:
        inputbag_basename, inputbag_ext = os.path.splitext(i)  # 分离文件名与扩展名
        outputbag_path = "cook_{}.bag".format(inputbag_basename)
        print(outputbag_path)
        if args.start_time is not None or args.end_time is not None:
            outputbag_path = "{}_cook_cut.bag".format(inputbag_basename)

    if os.path.exists(outputbag_path):
        overwrite = input("Output bag file already exists, overwrite? (y/n)")
        if overwrite != "y":
            print("Exiting...")
            return

    input_bag = rosbag.Bag(i, 'r')
    output_bag = rosbag.Bag(outputbag_path, 'w')
    start_time = rospy.Time.from_sec(args.start_time) if args.start_time is not None else None
    end_time = rospy.Time.from_sec(args.end_time) if args.end_time is not None else None

    # 将时间戳换为收到消息的时间戳
    with output_bag as outbag:
        print("Cooking...")
        message_count = input_bag.get_message_count()

        for topic, msg, t in tqdm(input_bag.read_messages(start_time=start_time, end_time=end_time),
                                  total=message_count):
            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            
            if not msg._has_header:
                # logging.warn("no msg header")
                # print(topic)
                # print(msg)
                outbag.write(topic, msg , msg.time)
            else:
                outbag.write(topic, msg, msg.header.stamp)
        print("Finished")
    input_bag.close()
    output_bag.close()


if __name__ == "__main__":
    main()
