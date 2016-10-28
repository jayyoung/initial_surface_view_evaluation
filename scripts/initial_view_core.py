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
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
from ptu_gaze_controller import *
from mongodb_store.message_store import MessageStoreProxy
import uuid
from initial_surface_view_evaluation.msg import *

class InitialViewEvaluationCore():
    def __init__(self):
        rospy.loginfo("waiting for services")
        self.conv_octomap = rospy.ServiceProxy('/surface_based_object_learning/convert_pcd_to_octomap',ConvertCloudToOctomap)
        self.get_normals = rospy.ServiceProxy('/surface_based_object_learning/extract_normals_from_octomap',ExtractNormalsFromOctomap)
        self.get_obs = rospy.ServiceProxy('/semantic_map_publisher/SemanticMapPublisher/ObservationService',ObservationService)
        self.roi_srv = rospy.ServiceProxy('/check_point_set_in_soma_roi',PointSetInROI)
        rospy.loginfo("done")
        self.ptu_gazer_controller = PTUGazeController()
        self.marker_publisher = rospy.Publisher("/initial_surface_view_evaluation/centroid", Marker)
        self.z_cutoff = 1
        self.obs_resolution = 0.03
        self.initial_view_store = MessageStoreProxy(database="initial_surface_views", collection="logged_views")


    def log_view(self,data):
        rospy.loginfo("logging initial view")
        waypoint,filtered_cloud,filtered_octomap,normals,segmented_objects_octomap = data
        lv = LoggedInitialView()
        lv.timestamp = int(rospy.Time.now().to_sec())
        lv.meta_data = "{}"
        lv.id = str(uuid.uuid4())
        lv.waypoint = waypoint
        lv.metaroom_filtered_cloud = filtered_cloud
        lv.metaroom_filtered_octomap = filtered_octomap
        lv.up_facing_points = normals
        
        #lv.segmented_objects_octomap = segmented_objects_octomap

        self.initial_view_store.insert_named(lv.id,lv)

    def do_task(self,waypoint):
        rospy.loginfo("-- Executing initial view evaluation task at waypoint: " + waypoint)
        obs = self.get_filtered_obs_from_wp(waypoint)
        octo_obs = self.convert_cloud_to_octomap(obs)
        normals = self.get_normals_from_octomap(octo_obs)

        rospy.loginfo("got: " + str(len(normals)) + " up-facing normal points")
        sx = 0
        sy = 0
        sz = 0
        for k in normals:
            sx+=k.x
            sy+=k.y
            sz+=k.z
        sx/=len(normals)
        sy/=len(normals)
        sz/=len(normals)
        centroid = [sx,sy,sz]
        print("centroid: " + str(centroid))
        centroid_marker = Marker()
        centroid_marker.header.frame_id = "/map"
        centroid_marker.type = Marker.SPHERE
        centroid_marker.header.stamp = rospy.Time.now()
        centroid_marker.pose.position.x = sx
        centroid_marker.pose.position.y = sy
        centroid_marker.pose.position.z = sz
        centroid_marker.scale.x = 0.1
        centroid_marker.scale.y = 0.1
        centroid_marker.scale.z = 0.1
        centroid_marker.color.a = 1.0
        centroid_marker.color.r = 1.0
        centroid_marker.color.g = 0.0
        centroid_marker.color.b = 0.0
        self.marker_publisher.publish(centroid_marker)
        fp = []
        for p in normals:
            fp.append([p.x,p.y,p.z,255])
        n_cloud = pc2.create_cloud(obs.header, obs.fields, fp)
        python_pcd.write_pcd("nrmls.pcd",n_cloud,overwrite=True)
        self.log_view([waypoint,obs,octo_obs,n_cloud,None])

        pt_s = PointStamped()
        pt_s.header.frame_id = "/map"
        # behind robot
        pt_s.point.x = sx
        pt_s.point.y = sy
        pt_s.point.z = sz
        self.ptu_gazer_controller.look_at_map_point(pt_s)
        self.do_view_sweep()


    def process_sweep_view(self):
        rospy.loginfo("- taking view")
        # segment the scene
        # run it through noise filters? what to do here?
        # return the segmented objects as a pc2


    def do_view_sweep(self):
        self.ptu_gazer_controller.pan_ptu_relative(45)
        rospy.sleep(1)
        self.ptu_gazer_controller.pan_ptu_relative(-45)
        rospy.sleep(1)
        self.ptu_gazer_controller.pan_ptu_relative(-45)
        rospy.sleep(1)
        self.ptu_gazer_controller.reset_gaze()

    def get_filtered_obs_from_wp(self,waypoint):
        rospy.loginfo("asking for latest obs at:" + waypoint)
        r = self.get_obs(waypoint,self.obs_resolution)
        rospy.loginfo("num points in this obs:" + str(len(r.cloud.data)))
        in_roi = 0
        out_roi = 0
        point_set = []
        raw_point_set = []
        rospy.loginfo("making point set")

        rp = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

        for p_in in pc2.read_points(r.cloud,field_names=["x","y","z","rgb"]):
            pp = geometry_msgs.msg.Point()
            pp.x = p_in[0]
            pp.y = p_in[1]
            pp.z = p_in[2]
            if(pp.z > self.z_cutoff and pp.z < 1.7):
                point_set.append(pp)
                raw_point_set.append(p_in)

        res = self.roi_srv(point_set,rp.position)
        print("done")
        print("size of point set: " + str(len(raw_point_set)))
        print("size of result: " + str(len(res.result)))
        print("points in roi: " + str(sum(res.result)))
        filtered_points = list(compress(raw_point_set,res.result))
        print("size of filtered: " + str(len(filtered_points)))
        rgb = pc2.create_cloud(r.cloud.header,r.cloud.fields,filtered_points)
        return rgb

    def convert_cloud_to_octomap(self,cloud):
        octo = self.conv_octomap(cloud)
        return octo.octomap

    def get_normals_from_octomap(self,octomap):
        norm = self.get_normals(octomap)
        return norm.up_facing_points

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    i = InitialViewEvaluationCore()
    i.do_task("WayPoint1")
