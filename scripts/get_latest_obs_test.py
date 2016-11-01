import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.srv import *


if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
    send_pcd = rospy.ServiceProxy('/surface_based_object_learning/convert_pcd_to_octomap',ConvertCloudToOctomap)
    print("done")

    cloud = rospy.wait_for_message("/head_xtion/depth_registered/points",PointCloud2)
    out = send_pcd([cloud])

    print("done")

    if(out is not None):
        print("seem to have gotten a response that isn't junk")

    #rospy.spin()
