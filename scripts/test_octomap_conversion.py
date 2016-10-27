import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from semantic_map_publisher.srv import *
from semantic_map.srv import *
from octomap_msgs.msg import *
from surface_based_object_learning.srv import *
from initial_surface_view_evaluation.srv import *
import sensor_msgs.point_cloud2 as pc2
from itertools import compress
import python_pcd

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")

    # callback chain to deal with storing *objects*
    print("waiting for services")
    conv_octomap = rospy.ServiceProxy('/surface_based_object_learning/convert_pcd_to_octomap',ConvertCloudToOctomap)
    get_normals = rospy.ServiceProxy('/surface_based_object_learning/extract_normals_from_octomap',ExtractNormalsFromOctomap)
    get_obs = rospy.ServiceProxy('/semantic_map_publisher/SemanticMapPublisher/ObservationService',ObservationService)
    roi_srv = rospy.ServiceProxy('/check_point_set_in_soma_roi',PointSetInROI)
    print("done")

    targ = "WayPoint1"
    print("asking for latest obs at:" + targ)
    r = get_obs(targ,0.05)
    print(r.cloud.header)
    print("num points in this map:" + str(len(r.cloud.data)))
    print("got it!")
    in_roi = 0
    out_roi = 0
    point_set = []
    raw_point_set = []
    print("making point set")

    rp = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)
    print(rp)

    for p_in in pc2.read_points(r.cloud,field_names=["x","y","z","rgb"]):
        pp = geometry_msgs.msg.Point()
        pp.x = p_in[0]
        pp.y = p_in[1]
        pp.z = p_in[2]
        if(pp.z > 1):
            point_set.append(pp)
            raw_point_set.append(p_in)

    res = roi_srv(point_set,rp.position)
    print("done")
    print("size of point set: " + str(len(raw_point_set)))
    print("size of result: " + str(len(res.result)))
    print("points in roi: " + str(sum(res.result)))

    filtered_points = list(compress(raw_point_set,res.result))
    print("size of filtered: " + str(len(filtered_points)))

    rgb = pc2.create_cloud(r.cloud.header,r.cloud.fields,filtered_points)
    #python_pcd.write_pcd("test.pcd",rgb,overwrite=True)
    octo = conv_octomap(rgb)
    print("getting normals")
    print(octo.octomap.header)
    nrmsl = get_normals(octo.octomap)
    print("all done")
    rospy.spin()
