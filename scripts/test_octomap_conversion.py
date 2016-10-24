import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from semantic_map_publisher.srv import *
from semantic_map.srv import *
from octomap_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
    get_octomap = rospy.ServiceProxy('SemanticMapPublisher/ObservationOctomapService',ObservationOctomapService)
    print("done")
