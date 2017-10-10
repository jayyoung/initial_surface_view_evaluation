#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from semantic_map_publisher.srv import *
import sys
import argparse
from semantic_map.srv import *
from octomap_msgs.msg import *
from surface_based_object_learning.srv import *
from initial_surface_view_evaluation.srv import *
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf, tf2_msgs.msg
from tf import TransformListener
from itertools import compress
#import python_pcd
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Pose, Transform, TransformStamped, Vector3, Quaternion
from ptu_gaze_controller import *
from mongodb_store.message_store import MessageStoreProxy
import uuid
from initial_surface_view_evaluation.msg import *
from segmentation_srv_definitions.srv import * # vienna seg
import PyKDL
import actionlib
from seg_filter.srv import *
import cv2
import image_geometry
from cv_bridge import CvBridge, CvBridgeError


class SegmentationWrapper():
    def __init__(self,cam):
        rospy.loginfo("VIEW EVAL: getting segmentation srv")
        self.segmentation_srv = rospy.ServiceProxy("/filtered_segmentation/segment", seg, 10)
        rospy.loginfo("VIEW EVAL: done")
        self.do_interest_filter = False
        self.interest_threshold = 20
        self.surf_filter = cv2.SURF(1000)
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_msg = rospy.wait_for_message(cam,  CameraInfo, timeout=2)
        rospy.loginfo("Got the following camera info:")
        rospy.loginfo(self.camera_msg)
        self.camera_model.fromCameraInfo(self.camera_msg)
        self.bridge = CvBridge()


    def segment(self,input_cloud,input_image):
        rospy.loginfo("VIEW EVAL: segmenting")
        # segment scene
        input_image = self.bridge.imgmsg_to_cv2(input_image)
        rospy.wait_for_service('/get_closest_roi_to_robot',10)
        roicl = rospy.ServiceProxy('/get_closest_roi_to_robot',GetROIClosestToRobot)
        #rospy.wait_for_service('/object_interestingness_estimator/estimate',10)
        #interest_srv = rospy.ServiceProxy('/object_interestingness_estimator/estimate',EstimateInterest)
        rp = rospy.wait_for_message("/robot_pose",Pose,10)
        roip = roicl(pose=rp.position)
        rospy.loginfo("VIEW EVAL: Beginning")
        output = self.segmentation_srv(cloud=input_cloud,posearray=roip.output)
        clusters = output.clusters_indices
        # return segments as a list of pointcloud2 objects
        raw_cloud = pc2.read_points(input_cloud)
        int_data = list(raw_cloud)
        aggregated_cloud = []
        rgb_points = []
        rospy.loginfo("Processing Results")
        ## quick and dirty, fix up later ##
        rgb_min_x = 90000
        rgb_max_x = 0
        rgb_min_y = 90000
        rgb_max_y = 0
        added_clusters = 0
        for c in clusters:
            rospy.loginfo("Evaluating Cluster")
            if(len(c.data) >= 200 and len(c.data) < 15000):
                t_cloud = []
                is_interesting = False

                for i in c.data:
                    point = int_data[i]
                    rgb_point = self.camera_model.project3dToPixel((point[0], point[1], point[2]))
                    rgb_x = int(rgb_point[0])
                    rgb_y = int(rgb_point[1])
                    if(rgb_x < rgb_min_x):
                        rgb_min_x = rgb_x
                    if(rgb_y < rgb_min_y):
                        rgb_min_y = rgb_y
                    if(rgb_x > rgb_max_x):
                        rgb_max_x = rgb_x
                    if(rgb_y > rgb_max_y):
                        rgb_max_y = rgb_y
                padding = 48
                y_start = rgb_min_y-padding
                y_end = rgb_max_y+padding
                x_start = rgb_min_x-padding
                x_end = rgb_max_x+padding
                if(y_end > 480):
                    y_end = 480
                if(x_end > 640):
                    x_end = 640
                if(y_start < 0):
                    y_start = 0
                if(x_start < 0):
                    x_start = 0
                segment_image_cropped = input_image[int(y_start):int(y_end), int(x_start):int(x_end)]

                kp, des = self.surf_filter.detectAndCompute(segment_image_cropped,None)
                rospy.loginfo("kp:" + str(len(kp)))
                if(len(kp) >= self.interest_threshold):
                    is_interesting = True

                if(is_interesting):
                    added_clusters+=1
                    for i in c.data:
                        aggregated_cloud.append(int_data[i])


        rgb = pc2.create_cloud(input_cloud.header,input_cloud.fields,aggregated_cloud)
        rospy.loginfo("VIEW EVAL: added " + str(added_clusters) + " clusters")
        #python_pcd.write_pcd("view.pcd",rgb,overwrite=True)
        rospy.loginfo("VIEW EVAL: done")
        return rgb

class InitialViewEvaluationCore():
    def __init__(self,cam):
        rospy.init_node('initial_surface_view_evaluation_actionserver', anonymous = False)
        rospy.loginfo("VIEW EVAL: waiting for services")
        self.conv_octomap = rospy.ServiceProxy('/surface_based_object_learning/convert_pcd_to_octomap',ConvertCloudToOctomap)
        self.get_normals = rospy.ServiceProxy('/surface_based_object_learning/extract_normals_from_octomap',ExtractNormalsFromOctomap)
        self.get_obs = rospy.ServiceProxy('/semantic_map_publisher/SemanticMapPublisher/ObservationService',ObservationService)
        self.roi_srv = rospy.ServiceProxy('/check_point_set_in_soma_roi',PointSetInROI)
        #self.gaze_srv = rospy.ServiceProxy('/surface_based_object_learning/gaze_at_map_point',GazeAtMapPoint)
        self.gaze_srv = rospy.Service('/surface_based_object_learning/gaze_at_map_point',GazeAtMapPoint,self.gaze_cb)

        self.closest_roi_srv = rospy.ServiceProxy('/get_closest_roi_to_robot',GetROIClosestToRobot)
        #self.overlap_srv = rospy.ServiceProxy('/surface_based_object_learning/calculate_octree_overlap',CalculateOctreeOverlap)
        rospy.loginfo("VIEW EVAL: done")

        self.pc_topic = "/head_xtion/depth_registered/points"
        #self.pc_topic = "/head"

        self.rgb_topic = "/head_xtion/rgb/image_color"

        self.ptu_gazer_controller = PTUGazeController()
        self.marker_publisher = rospy.Publisher("/initial_surface_view_evaluation/centroid", Marker,queue_size=5)
        self.min_z_cutoff = 0.7
        self.max_z_cutoff = 1.7
        self.obs_resolution = 0.03
        self.initial_view_store = MessageStoreProxy(database="initial_surface_views", collection="logged_views")
        self.segmentation = SegmentationWrapper(cam)
        self.transformation_store = TransformListener()
        rospy.sleep(5)
        self.action_server = actionlib.SimpleActionServer("/surface_based_object_learning/evaluate_surface", EvaluateSurfaceAction,
        execute_cb=self.do_task_cb, auto_start = False)
        self.action_server.start()
        rospy.loginfo("VIEW EVAL: action server set up")
        rospy.loginfo("VIEW EVAL: done")
        rospy.spin()


    def log_task(self,data):
        rospy.loginfo("VIEW EVAL: logging initial view")
        waypoint,filtered_octomap,normals,segmented_objects_octomap,sweep_imgs = data
        lv = LoggedInitialView()
        lv.timestamp = int(rospy.Time.now().to_sec())
        lv.meta_data = "{}"
        lv.id = str(uuid.uuid4())
        lv.waypoint = waypoint
        #lv.metaroom_filtered_cloud = filtered_cloud
        lv.metaroom_filtered_octomap = filtered_octomap
        lv.up_facing_points = normals
        lv.segmented_objects_octomap = segmented_objects_octomap
        #lv.sweep_clouds = sweep_clouds
        lv.sweep_imgs = sweep_imgs
        self.initial_view_store.insert_named(lv.id,lv)
        rospy.loginfo("VIEW EVAL: done")

    def do_task_cb(self,action):
        octo = self.do_task(action.waypoint_id)
        re =  initial_surface_view_evaluation.msg.EvaluateSurfaceResult(octomap=octo)
        self.action_server.set_succeeded(re)

    def do_task(self,waypoint):
        try:
            rospy.loginfo("VIEW EVAL: -- Executing initial view evaluation task at waypoint: " + waypoint)
            obs = self.get_filtered_obs_from_wp(waypoint)
            octo_obs = self.convert_cloud_to_octomap([obs])
            normals = self.get_normals_from_octomap(octo_obs)

            rospy.loginfo("VIEW EVAL: got: " + str(len(normals)) + " up-facing normal points")
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
            #python_pcd.write_pcd("nrmls.pcd",n_cloud,overwrite=True)

            pt_s = PointStamped()
            pt_s.header.frame_id = "/map"
            # behind robot
            pt_s.point.x = sx
            pt_s.point.y = sy
            pt_s.point.z = sz
            segmented_clouds,sweep_clouds,sweep_imgs = self.do_view_sweep_from_point(pt_s)
            roi_filtered_objects = self.get_filtered_roi_cloud(segmented_clouds)
            object_octomap = self.convert_cloud_to_octomap([roi_filtered_objects])
            #object_octomap.header = "/map"
            # waypoint,filtered_cloud,filtered_octomap,normals,segmented_objects_octomap,sweep_clouds,sweep_imgs
            self.log_task([waypoint,octo_obs,n_cloud,object_octomap,sweep_imgs])
            return object_octomap
        except Exception,e:
            rospy.logerr(e)
            rospy.sleep(1)
            self.ptu_gazer_controller.reset_gaze()
            rospy.sleep(1)
            rospy.logerr("PTU has been reset")

    # includes a bunch of sleeps just to make super extra sure we don't get any camera blur due to all the movement

    def gaze_cb(self,point):
        rospy.loginfo("Gazing at: " + str(point))
        self.ptu_gazer_controller.reset_gaze()
        rospy.sleep(2)
        self.ptu_gazer_controller.look_at_map_point(point)
        rospy.sleep(2)
        result = GazeAtMapPointResponse(True)
        return result

    def do_view_sweep_from_point(self,point):

        rospy.loginfo("VIEW EVAL: doing mini-sweep")
        sweep_clouds = []
        sweep_imgs = []
        segmented_clouds = []
        sweep_degree = 15
        angles = [sweep_degree,-sweep_degree,-sweep_degree]

        self.ptu_gazer_controller.reset_gaze()

        rospy.sleep(2)

        self.ptu_gazer_controller.look_at_map_point(point)

        rospy.sleep(2)

        for a in angles:
            self.ptu_gazer_controller.pan_ptu_relative(a)
            #rospy.loginfo("sleeping to give temporal smoothing something to look at")
            rospy.sleep(2)
            try:
                image = rospy.wait_for_message(self.rgb_topic,Image,timeout=10.0)
                sweep_imgs.append(image)
                cloud = rospy.wait_for_message(self.pc_topic,PointCloud2,timeout=10.0)
                segmented_cloud = self.segmentation.segment(cloud,image)
                sweep_clouds.append(cloud)
                segmented_map_cloud = self.transform_cloud_to_map(segmented_cloud)
                segmented_clouds.append(segmented_map_cloud)
            except Exception,e:
                rospy.logerr("Unable to perform this view")
                rospy.logerr(e)

        rospy.sleep(2)
        self.ptu_gazer_controller.reset_gaze()
        return segmented_clouds,sweep_clouds,sweep_imgs

    def get_filtered_roi_cloud(self,cloud_set):
        point_set = []
        raw_point_set = []
        rospy.loginfo("VIEW EVAL: making point set to evaluate segmentation clouds against ROI")
        rp = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

        for cloud in cloud_set:
            rospy.loginfo("merging cloud and cropping to ROI")
            for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
                pp = geometry_msgs.msg.Point()
                pp.x = p_in[0]
                pp.y = p_in[1]
                pp.z = p_in[2]

                # should be in map coords by here
                if(pp.z > self.min_z_cutoff and pp.z < self.max_z_cutoff):
                    point_set.append(pp)
                    raw_point_set.append(p_in)

        res = self.roi_srv(point_set,rp.position)
        rospy.loginfo("done")
        rospy.loginfo("size of point set: " + str(len(raw_point_set)))
        rospy.loginfo("size of result: " + str(len(res.result)))
        rospy.loginfo("points in roi: " + str(sum(res.result)))
        filtered_points = list(compress(raw_point_set,res.result))
        rospy.loginfo("size of filtered: " + str(len(filtered_points)))
        rgb = pc2.create_cloud(cloud_set[0].header,cloud_set[0].fields,filtered_points)
        return rgb

    def get_filtered_obs_from_wp(self,waypoint):
        rospy.loginfo("VIEW EVAL: asking for latest obs at:" + waypoint)
        r = self.get_obs(waypoint,self.obs_resolution)
        rospy.loginfo("VIEW EVAL: num points in this obs:" + str(len(r.cloud.data)))
        in_roi = 0
        out_roi = 0
        point_set = []
        raw_point_set = []
        rospy.loginfo("VIEW EVAL: making point set")

        rp = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

        for p_in in pc2.read_points(r.cloud,field_names=["x","y","z","rgb"]):
            pp = geometry_msgs.msg.Point()
            pp.x = p_in[0]
            pp.y = p_in[1]
            pp.z = p_in[2]
            if(pp.z > self.min_z_cutoff and pp.z < self.max_z_cutoff):
                point_set.append(pp)
                raw_point_set.append(p_in)

	#rp.position.x = -9999
        rospy.loginfo("SENDING FILTER POINT: " + str(rp.position))
        res = self.roi_srv(point_set,rp.position)
        rospy.loginfo("done")
        rospy.loginfo("size of point set: " + str(len(raw_point_set)))
        rospy.loginfo("size of result: " + str(len(res.result)))
        rospy.loginfo("points in roi: " + str(sum(res.result)))
        filtered_points = list(compress(raw_point_set,res.result))
        rospy.loginfo("size of filtered: " + str(len(filtered_points)))
        rgb = pc2.create_cloud(r.cloud.header,r.cloud.fields,filtered_points)
        return rgb

    def convert_cloud_to_octomap(self,cloud):
        rospy.loginfo("Converting Cloud to Octomap")
        octo = self.conv_octomap(cloud)
        return octo.octomap

    def get_normals_from_octomap(self,octomap):
        norm = self.get_normals(octomap)
        return norm.up_facing_points

    def transform_cloud_to_map(self,cloud):
        rospy.loginfo("VIEW EVAL: to map from " + cloud.header.frame_id)
        if("temporal" in cloud.header.frame_id):
            rospy.loginfo("un-breaking this")
            cloud.header.frame_id = "head_xtion_depth_optical_frame"

        self.transformation_store = TransformListener()
        rospy.sleep(5)
        t = self.transformation_store.getLatestCommonTime("map", cloud.header.frame_id)
        tr_r = self.transformation_store.lookupTransform("map", cloud.header.frame_id, t)

        tr = Transform()
        tr.translation = Vector3(tr_r[0][0],tr_r[0][1],tr_r[0][2])
        tr.rotation = Quaternion(tr_r[1][0],tr_r[1][1],tr_r[1][2],tr_r[1][3])
        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = "map"
        tr_s.child_frame_id = cloud.header.frame_id
        tr_s.transform = tr
        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        rospy.loginfo("CLOUD FIELDS:")
        rospy.loginfo(cloud.fields)
        filtered_fields = []
        # christ, why. this is a hack to fix something with the temporal smoothed pc
        for k in cloud.fields:
            if(k.offset == 0):
                filtered_fields.append(k)
            if(k.offset == 4):
                filtered_fields.append(k)
            if(k.offset == 8):
                filtered_fields.append(k)
            if(k.offset == 7):
                filtered_fields.append(k)



        for p_in in pc2.read_points(cloud,field_names=["x","y","z","rgb"]):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2]])

        cloud.header.frame_id = "map"
        res = pc2.create_cloud(cloud.header, filtered_fields, points_out)
        rospy.loginfo(cloud.header)
        return res

    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='initial_view_core.py')
    parser.add_argument('camera_info_topic', nargs=1, help="Camera Info Topic")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    cam = str(vars(args)['camera_info_topic'][0])


    i = InitialViewEvaluationCore(cam)
    #i.do_task("WayPoint1")
    #s = SegmentationWrapper()
    #cl = rospy.wait_for_message("/head_xtion/depth/points",PointCloud2,timeout=10.0)
    #s.segment(cl)
    #rospy.spin()
