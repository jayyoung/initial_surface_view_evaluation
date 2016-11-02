import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.srv import *
from segmentation_srv_definitions.srv import * # vienna seg
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf, tf2_msgs.msg
from tf import TransformListener
import PyKDL
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
#import python_pcd

def transform_cloud_to_map(cloud):
    rospy.loginfo("to map from " + cloud.header.frame_id)

    transformation_store = TransformListener()
    rospy.sleep(2)

    t = transformation_store.getLatestCommonTime("map", cloud.header.frame_id)
    tr_r = transformation_store.lookupTransform("map", cloud.header.frame_id, t)


    tr = Transform()
    tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
    tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
    tr_s = TransformStamped()
    tr_s.header = std_msgs.msg.Header()
    tr_s.header.stamp = rospy.Time.now()
    tr_s.header.frame_id = "map"
    tr_s.child_frame_id = cloud.header.frame_id
    tr_s.transform = tr
    t_kdl = transform_to_kdl(tr_s)
    points_out = []
    for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        points_out.append([p_out[0],p_out[1],p_out[2]])

    cloud.header.frame_id = "map"
    res = pc2.create_cloud(cloud.header, cloud.fields, points_out)
    rospy.loginfo(cloud.header)
    return res

def transform_to_kdl(t):
     return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                  t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z))



if __name__ == '__main__':
    rospy.init_node('overa_test', anonymous = False)

    #print("waiting for pc topic")
    #rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
    #print("got it")
    overlap_srv = rospy.ServiceProxy('/surface_based_object_learning/calculate_octree_overlap',CalculateOctreeOverlap)
    input_cloud = rospy.wait_for_message("/head_xtion/depth/points",PointCloud2,timeout=10.0)
    segmentation_srv = rospy.ServiceProxy("/pcl_segmentation_service/pcl_segmentation", segment, 10)

    output = segmentation_srv(cloud=input_cloud)
    clusters = output.clusters_indices
    raw_cloud = pc2.read_points(input_cloud)
    int_data = list(raw_cloud)
    clouds = []

    for c in clusters:
        new_cloud = []
        for i in c.data:
            new_cloud.append(int_data[i])
        clouds.append(new_cloud)

    seg_clouds = []
    input_cloud.header.frame_id = "map"
    for c in clouds:
        print("seg")
        rgb = pc2.create_cloud(input_cloud.header,input_cloud.fields,c)
        map_rgb = transform_cloud_to_map(rgb)
        seg_clouds.append(map_rgb)
        #python_pcd.write_pcd(""+str(clouds.index(c))+".pcd",map_rgb)

    print("size: "+ str(len(seg_clouds)))

    for s in seg_clouds:
        for x in seg_clouds:
            o = overlap_srv(s,x)
            print(str(seg_clouds.index(s)) + " : " + str(seg_clouds.index(x)) + " == "+str(o))

    #rospy.spin()
