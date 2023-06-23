#! /usr/bin/python3

import rospy
from std_srvs.srv import SetBoolRequest, SetBool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from camera_control_msgs.srv import SetExposure, SetExposureRequest



rospy.init_node("flash_test")

# todo? check if auto_flash is activated for camera

camera_name = rospy.get_param("~camera_name", "/pylon_camera_node")
exposure = rospy.get_param("~exposure", 10000)
img_topic = camera_name + '/image_raw'
bridge = CvBridge()


# ensure current value (e.g. defaults) are visible on parameter server
rospy.set_param("~exposure", exposure)
rospy.set_param("~camera_name", camera_name)

if exposure > 0:
    rospy.loginfo("Setting exposure to %i ns" % exposure)
    exp_client = rospy.ServiceProxy(camera_name+'/set_exposure', SetExposure)
    try:
        exp_client.wait_for_service(3)
    except Exception as e:
       rospy.logerr("%s, terminating" % str(e))
       exit(1)
    print(exp_client.call(target_exposure=exposure))

# establish service clients for outputs
clients = list()
for i in [0, 1]:
    client = rospy.ServiceProxy(camera_name + "/activate_autoflash_output_" + str(i), SetBool)
    try:
        client.wait_for_service(3)
    except Exception as e:
        rospy.logerr("%s, terminating" % str(e))
        exit(1)
    clients.append(client)

assert len(clients) == 2


# Create different light situations and capture image (turn both off at end to stop the party)
lights = [(1, 1), (0, 1), (1, 0), (0, 0)]

img_prefix = "/tmp/flash_test_"

rospy.loginfo("Writing images to %s_*.png", img_prefix)

for light in lights:
    rospy.loginfo("Setting lights to " + str(light))
    print(clients[0].call(bool(light[0])))
    rospy.sleep(0.02)
    print(clients[1].call(bool(light[1])))
    rospy.sleep(0.5) # needed??

    # continue
    try:
      img = rospy.wait_for_message(img_topic, Image, 2)
    except rospy.exceptions.ROSException as e:
        rospy.logerr("Did not receive image at %s", img_topic)
        continue

    cv_img = bridge.imgmsg_to_cv2(img)
    print(cv_img.shape)
    filename = img_prefix + "%i_%i.png" % (light[0], light[1])
    cv2.imwrite(filename, cv_img)
    # rospy.sleep(2)


rospy.loginfo("Test ended")















