#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse
import numpy as np
import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from aml_robot.baxter_robot import BaxterArm
import quaternion
from baxter_pykdl import baxter_kinematics

def map_keyboard():
    left = baxter_interface.Limb('left')
    # left = BaxterArm('left')
    right = BaxterArm('right')

    kin = baxter_kinematics('left')

    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()
    leftdes = []
    leftacc = []
    lefterr = []
    delta = 0.075 #was 0.05
    gain = 1.

    def IK(limb, movement):
        # current_limb= limb.angles()
        # joint_command = {joint_name: current_position + delta}
        # current_pos, curr_ori = limb.ee_pose()

        # leftacc = np.append(current_pos,quaternion.as_euler_angles(curr_ori))
        # print leftacc.shape
        joint_names = limb.joint_names()
        # curr_q = limb.joint_angles()

        def to_list(ls):
            return [ls[n] for n in joint_names]

        
        curr_q = np.asarray(to_list(limb.joint_angles())).reshape([7,1])

            # print curr_q.shape, 'shape'
            # print 'ee pose', limb.endpoint_pose()

        pos = np.array(limb.endpoint_pose()['position'])
	
            #ori = limb.endpoint_pose()['orientation']

            #pose = np.append(pos, quaternion.as_euler_angles(quaternion.quaternion(ori.w,ori.x,ori.y,ori.z))).reshape([6,1])
            #print pose
            # print pos, ori
            # sldfjsl
            # raw_input()
        JT = kin.jacobian_transpose()

        lefterr = np.append(movement,[0,0,0]).reshape([6,1])
        #lefterr = np.array(movement).reshape([3,1])
            # leftdes = leftacc+lefterr
            # print leftacc, lefterr, leftdes
            #JT = np.linalg.pinv(limb.jacobian())
            # JT = limb.jacobian().T
        des_q = curr_q + gain*JT.dot(lefterr)
	
	
            #des_q = JT.dot(lefterr + pose)
        print des_q.shape, 'shape2' 
        q_dict ={}
        for i in range(len(joint_names)):

            q_dict[joint_names[i]] = des_q[i,:][0]
                # print joint_names[i], q_dict[joint_names[i]]
                # dsflasjkds
          
            # joint_command = current_limb + JT.dot(lefterr)
           # joint_command = JT.dot(leftdes)
            # print joint_command
           # print JT.dot(lefterr)
            # limb.exec_position_cmd(joint_command)
            # joint_command = {joint_name: curr_q + JT.dot(lefterr)}
            # print q_dict
        limb.move_to_joint_positions(q_dict)
	#print (pos[1]-np.array(limb.endpoint_pose()['position'])[1])/delta 0.91 for x, 0.82 for z
	print (pos[2]-np.array(limb.endpoint_pose()['position'])[2])/delta


    bindings = {
    #   key: (function, args, description)
      #  's': (IK, [left, [delta,delta/100,delta/100] ], "xinc"),
	#'d': (IK, [left, [-delta,-delta/100,-delta/100] ], "xdec"),
	's': (IK, [left, [delta*0.8,0,0] ], "xinc"),
        'd': (IK, [left, [-delta*0.8,0,0] ], "xdec"),
        'w': (IK, [left, [0,delta*0.5,0] ], "yinc"),
        'e': (IK, [left, [0,-delta*0.5,0] ], "ydec"),
	#'w': (IK, [left, [-delta/20,delta,-delta/20] ], "yinc"),
     #   'e': (IK, [left, [delta/20,-delta,delta/20] ], "ydec"),
        'x': (IK, [left, [0,0,delta] ], "zinc"),
        'c': (IK, [left, [0,0,-delta] ], "zdec"),
	'k': (IK, [left, [0,0,0] ], "nothing"),
	'l': (IK, [left, [delta/100,delta,delta/100] ], "allincy"),
	';': (IK, [left, [-delta,-delta,-delta] ], "alldec"),

        #'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        'b': (grip_left.close, [], "left: gripper close"),
        'n': (grip_left.open, [], "left: gripper open"),
        'm': (grip_left.calibrate, [], "left: gripper calibrate") #comma here?
     }

 
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
        


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
