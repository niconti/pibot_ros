#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Nicola Conti

import os
import sys
import string
# ROS
import rclpy
import rclpy.node
import rclpy.logging
import sensor_msgs.msg
# Pan-Tilt
from pibot_ros.PCA9685 import PCA9685

MIN_PAN = 10
MAX_PAN = 170
MIN_TILT = 0
MAX_TILT = 80


class CameraTeleop(rclpy.node.Node):

    def __init__(self):
        super().__init__('camera_teleop_node')

        self.pan = (MAX_PAN - MIN_PAN) / 2
        self.tilt = (MAX_TILT - MIN_TILT) / 2

        # Subscribed Topics
        self.joy_sub = self.create_subscription(sensor_msgs.msg.Joy, '/joy', self.joy_cb, 10)

    def joy_cb(self, msg):

        self.get_logger().debug("axes: {}".format(msg.axes))

        pan_inc  = round(msg.axes[0])
        tilt_inc = round(msg.axes[1])

        self.pan += pan_inc
        self.pan = max(self.pan, MIN_PAN)
        self.pan = min(MAX_PAN, self.pan)

        self.tilt += tilt_inc
        self.tilt = max(self.tilt, MIN_TILT)
        self.tilt = min(MAX_TILT, self.tilt)

        self.get_logger().info("pan, tilt: {} {}".format(self.pan, self.tilt))


def main(args=None):
    rclpy.init(args=args)

    # Node
    node = CameraTeleop()

    # Init PCA9685
    try:
        pwm = PCA9685()
        pwm.setPWMFreq(50)
    except OSError as ex:
        node.get_logger().error("{}".format(ex))
        exit(1)

    # rate = node.create_rate(10)
    while rclpy.ok():

        pwm.setRotationAngle(1, node.pan)
        pwm.setRotationAngle(0, node.tilt)

        rclpy.spin_once(node)

    pwm.exit_PCA9685()


if __name__ == '__main__':
    main()
