import roslib
import rospy
import actionlib
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.srv import *
from initial_surface_view_evaluation.msg import *


if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)
    rospy.loginfo("Waiting for initial surface view evaluation")
    eval_action_name = "/surface_based_object_learning/evaluate_surface"
    client = actionlib.SimpleActionClient(eval_action_name, EvaluateSurfaceAction)
    client.wait_for_server(rospy.Duration(60))
    goal = EvaluateSurfaceGoal(waypoint_id="WayPoint1")
    client.send_goal(goal)
    client.wait_for_result()
    print("done")
    octomap = client.get_result()
    print(octomap)
