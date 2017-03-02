import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.srv import *
from semantic_map_publisher.srv import *
from semantic_map.srv import *
from surface_based_object_learning.srv import *

if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for service")
   

    get_obs = rospy.ServiceProxy('/semantic_map_publisher/SemanticMapPublisher/ObservationService',ObservationService)
    obs = get_obs("WayPoint17",0.03)    
    print("got something" + str(obs.cloud.header))
    print("num points in cloud: " + str(len(obs.cloud.data)))

    roi_srv = rospy.ServiceProxy('/check_point_set_in_soma_roi',PointSetInROI)


    rp = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

    roi_srv(    


    #rospy.spin()
