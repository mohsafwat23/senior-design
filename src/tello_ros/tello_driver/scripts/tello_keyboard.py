#!/usr/bin/env python3
...
"""
author: Mohamed Safwat
email: mohamedmohabsafwat@gmail.com
"""
# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import sys

import geometry_msgs.msg
import rclpy
from tello_msgs.srv import TelloAction
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnEntity

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    w    o
   d    s    a
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
z : up (+z)
x : down (-z)
anything else : stop
m/n : increase/decrease max speeds by 10%
b/v : increase/decrease only linear speed by 10%
>/< : increase/decrease only angular speed by 10%
CTRL-C to quit
"""
actionBindings = {
    't': 'takeoff',
    'q': 'land',
    'p': 'reset',
}

moveBindings = {
    #Forward/Backward
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    #Left/Right
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    #Up/Down
    'z': (0, 0, 1, 0),
    'x': (0, 0, -1, 0),
    #Rotate Left/Right
    'e': (0, 0, 0, 1),
    'r': (0, 0, 0, -1),
    #Rotate Forward/Backward
    'o': (1, 0, 0, -1),
    'i': (1, 0, 0, 1),
    'l': (-1, 0, 0, -1),
    'k': (-1, 0, 0, 1),
}

speedBindings = {
    'm': (1.1, 1.1),
    'n': (.9, .9),
    'b': (1.1, 1),
    'v': (.9, 1),
    '>': (1, 1.1),
    '<': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)
    drone_action_client = node.create_client(TelloAction, '/tello_action')
    #spawn_client = node.create_client(SpawnEntity, 'spawn_entity')

    speed = 0.2#0.5
    turn = 0.2 #1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0 

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key in actionBindings.keys():
                if actionBindings[key] == 'takeoff':
                    print(actionBindings[key])
                    request = TelloAction.Request()
                    request.cmd = actionBindings[key]
                    drone_action_client.call_async(request)
                elif actionBindings[key] == 'land':
                    print(actionBindings[key])
                    request = TelloAction.Request()
                    request.cmd = actionBindings[key]
                    drone_action_client.call_async(request)
                # elif actionBindings[key] == 'reset':
                #     print(actionBindings[key])
                #     reset_world()

            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()