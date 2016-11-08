import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from initial_surface_view_evaluation.msg import *
from mongodb_store.message_store import MessageStoreProxy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
import cv
import cv2
import os
if __name__ == '__main__':
    rospy.init_node('om_test', anonymous = False)

    bridge = CvBridge()
    store = MessageStoreProxy(database="initial_surface_views", collection="logged_views")
    scenes = store.query(LoggedInitialView._type)
    print("got some scenes:", len(scenes))
    if not os.path.exists("initial_views_dump/"):
        os.makedirs("initial_views_dump/")
    for sc in scenes:
        episode = sc[0].id
        print("writing views from: ",episode)
        if not os.path.exists("initial_views_dump/"+episode+"/"):
            os.makedirs("initial_views_dump/"+episode+"/")
        for img in sc[0].sweep_imgs:
            print("\t writing view")
            im = bridge.imgmsg_to_cv2(img)
            cv2.imwrite("initial_views_dump/"+episode+"/view_"+str(int(sc[0].sweep_imgs.index(img)))+'.jpeg',im)

    print("all done!")
