#!/usr/bin/env python
import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import topological_navigation.msg
import scitos_ptu.msg
from sensor_msgs.msg import JointState
import tf2_ros
import tf, tf2_msgs.msg
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
import math


class PTUGazeController:

        def __init__(self,ptu_speed):
            print("-- PTU gaze controller created")
	    self.ptu_speed = ptu_speed

        def reset_gaze(self):
            rospy.loginfo("Trying to reset gaze")
            ptuClient = actionlib.SimpleActionClient('ResetPtu',scitos_ptu.msg.PtuResetAction)
            ptuClient.wait_for_server()

            goal = scitos_ptu.msg.PtuResetGoal()
            ptuClient.send_goal(goal)
            ptuClient.wait_for_result()
            rospy.loginfo("Done")

        def pan_ptu_relative(self,pan):
            ptuClient = actionlib.SimpleActionClient('SetPTUState',scitos_ptu.msg.PtuGotoAction)
            ptuClient.wait_for_server()
            cur_ptu_state = rospy.wait_for_message("/ptu/state",  JointState, timeout=10)
            goal = scitos_ptu.msg.PtuGotoGoal()
            goal.tilt = math.degrees(cur_ptu_state.position[1])
            goal.tilt_vel = self.ptu_speed
            goal.pan = math.degrees(cur_ptu_state.position[0])+pan
            goal.pan_vel = self.ptu_speed
            ptuClient.send_goal(goal)
            ptuClient.wait_for_result()
            rospy.loginfo("done")


        def look_at_map_point(self,point):
            self.reset_gaze()
            rospy.loginfo("looking at: ")
            rospy.loginfo(str(point.point))
            pan,tilt = self.transform_target_point(point)

            ptuClient = actionlib.SimpleActionClient('SetPTUState',scitos_ptu.msg.PtuGotoAction)
            ptuClient.wait_for_server()
            goal = scitos_ptu.msg.PtuGotoGoal()
            goal.tilt = tilt
            goal.tilt_vel = self.ptu_speed
            goal.pan = pan
            goal.pan_vel = self.ptu_speed
            ptuClient.send_goal(goal)
            ptuClient.wait_for_result()
            rospy.loginfo("done")

        def transform_target_point(self,target):
            pan_ref_frame = '/head_xtion_link'
            tilt_ref_frame = '/head_xtion_link'

            tfs = tf.TransformListener()
            # Wait for tf info (timeout in 5 seconds)
            tfs.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))
            tfs.waitForTransform(tilt_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))

            # Transform target point to pan reference frame & retrieve the pan angle
            pan_target = tfs.transformPoint(pan_ref_frame, target)
            pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)
            #print("pan angle: " + str(pan_angle))

            # Transform target point to tilt reference frame & retrieve the tilt angle
            tilt_target = tfs.transformPoint(tilt_ref_frame, target)
            tilt_angle = math.atan2(-tilt_target.point.z,math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

            cur_ptu_state = rospy.wait_for_message("/ptu/state",  JointState, timeout=10)
            #print("cur ptu state: " + str(cur_ptu_state.position))
            current_head_pan = cur_ptu_state.position[0]
            current_head_tilt = cur_ptu_state.position[1]

            return [math.degrees(current_head_pan+pan_angle), math.degrees(current_head_tilt+tilt_angle)]

if __name__ == '__main__':
    rospy.init_node('ptu_gazer_controller', anonymous = True)
    pt_s = PointStamped()
    pt_s.header.frame_id = "/map"

    # behind robot
    pt_s.point.x = 5.1
    pt_s.point.y = 3.6
    pt_s.point.z = 3.48

    # in front of robot
    #pt_s.point.x = 9.09
    #pt_s.point.y = 2.94
    #pt_s.point.z = 1.62

    # to the side of the robot
    #pt_s.point.x = 7.38
    #pt_s.point.y = 1.74
    #pt_s.point.z = 4.19
    p = PTUGazeController(10)
    p.reset_gaze()
    rospy.sleep(3)
    p.look_at_map_point(pt_s)
    rospy.sleep(3)
    p.pan_ptu_relative(45)
    rospy.sleep(3)
    p.pan_ptu_relative(45)
    rospy.sleep(3)
    p.pan_ptu_relative(45)
    rospy.sleep(3)
