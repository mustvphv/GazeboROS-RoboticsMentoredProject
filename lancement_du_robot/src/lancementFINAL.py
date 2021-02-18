#!/usr/bin/python
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
from std_msgs.msg import Float32
from collections import deque
import argparse
import imutils
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2

#le programme a été fait en s'aidant des liens suivant:
#https://stackoverflow.com/questions/59131771/how-to-read-a-frame-raw-image-from-a-virtual-turtlebots-camera-in-gazebo-and
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/
#https://answers.gazebosim.org//question/3336/change-color-of-models/
#http://gazebosim.org/tutorials?tut=build_world
#https://www.semicolonworld.com/question/57841/how-to-find-the-red-color-regions-using-opencv
#https://answers.opencv.org/question/90047/detecting-blue-color-in-this-image/
#Un grand nombre d’articles issus de site Internet ont servi pour l’apprentissage de ROS et Gazebo,
#Pour la reconaissance de couleur, on s’est beaucoup aidé de la documentation fournie par le site https://www.theconstructsim.com ainsi que par leurs chaîne Youtube: https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q .
#-On a aussi utilisé les tutoriels et les questions-réponses de http://gazebosim.orget de https://answers.gazebosim.org/questions/.
#-De plus, https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/ .
#-https://www.theconstructsim.com/exploring-ros-2-wheeled-robot-part-5/
#Pour Blender : 
#-https://www.youtube.com/watch?v=l15ro77fnEM
#-https://www.youtube.com/watch?v=r5YNJghc81U



global shutdown_var
global couleur_bleue_trouvee
global else_active






def image_callback(msg):
    global shutdown_var

    global couleur_bleue_trouvee

    global else_active

    bridge = CvBridge()

    try:

        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:

        try:
            os.remove('/home/ilyes/projet_ws/src/lancement_du_robot/src/live_camera1_images/camera_image.jpeg')
        except: pass

        
        cv2.imwrite('/home/ilyes/projet_ws/src/lancement_du_robot/src/live_camera1_images/camera_image.jpeg', cv2_img)

        
        
        image = cv2.imread('/home/ilyes/projet_ws/src/lancement_du_robot/src/live_camera1_images/camera_image.jpeg', cv2.IMREAD_COLOR)

        height, width, channels = cv2_img.shape

        descentre = 160

        rows_to_watch = 60

        velocity_publisher = rospy.Publisher('/the_robot/cmd_vel', Twist, queue_size=10)


        img_hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        blue_lower = np.array([100, 150, 0], np.uint8)
        blue_upper = np.array([140, 255, 255], np.uint8)


        vel_msg = Twist();


        image = imutils.resize(image, width=600)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None


        print("couleur_bleue_trouvee", couleur_bleue_trouvee)
        if len(cnts) > 0 and couleur_bleue_trouvee == 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


            print('(int(M["m01"] / M["m00"])=', (int(M["m01"] / M["m00"])))


            error_x = int(M['m10']/M['m00']) - 600 / 2;

            vel_msg.angular.z = -error_x / 100;

            print("DE NOUVEAU")



            print('DANS IF >50 z angular=', vel_msg.angular.z)

            velocity_publisher.publish(vel_msg)

            
            if vel_msg.angular.z == 0:
                print("ANGULAR 00000")
                couleur_bleue_trouvee = 1
                #rospy.signal_shutdown("end /the_robot/cmd_vel")


        elif else_active == 1:
            print("dans le ELIF")


            speed = 0.5
            distance = 10
            isForward = True

            if(isForward):
                vel_msg.linear.x = abs(speed)
            else:
                vel_msg.linear.x = -abs(speed)

            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0


            while vel_msg.linear.x != 0:

                t0 = rospy.Time.now().to_sec()
                current_distance = 0

                while(current_distance < distance):
                    velocity_publisher.publish(vel_msg)

                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)


                vel_msg.linear.x = 0
                velocity_publisher.publish(vel_msg)

            else_active = 0
            couleur_bleue_trouvee = 0
            

        
        else:
            if couleur_bleue_trouvee == 1:
                print("dans le ELSE")

                vel_msg.linear.x = 0;
                vel_msg.angular.z = 0;
                velocity_publisher.publish(vel_msg)
                else_active = 1

        



def take_action(regions):
    global shutdown_var

    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 1 - nothing'
        #linear_x = 0.6
        #angular_z = 0
        #print("msg_t.ranges[360]=", msg_t.ranges[360])
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        #linear_x = 0
        #angular_z = -0.3
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        #linear_x = 0
        #angular_z = -0.3
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        #linear_x = 0
        #angular_z = 0.3
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        #linear_x = 0
        #angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        #linear_x = 0
        #angular_z = 0.3
    elif regions['front'] < 0.3 and regions['fleft'] < 0.3 and regions['fright'] < 0.3:
        state_description = 'case 7 - front and fleft and fright'
        print("DANS LE CAS 7")
        shutdown_var = 1
        #rospy.shu
        #linear_x = 0
        #angular_z = 0
    elif regions['front'] > 0.2 and regions['fleft'] < 0.2 and regions['fright'] < 0.2:
        state_description = 'case 8 - fleft and fright'
        print("DANS LE CAS 8")
        #shutdown_var = 1
        #linear_x = 0
        #angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    msg.linear.x = linear_x
    msg.angular.z = angular_z




def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action(regions)





def moveTemp():
    global shutdown_var
    global couleur_bleue_trouvee
    global else_active

    shutdown_var = 0
    couleur_bleue_trouvee = 0
    else_active = 0

    rospy.init_node('robot_search_blue_color', anonymous=True)
    velocity_publisher = rospy.Publisher('/the_robot/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()



    speed = 90 
    distance = 10
    isForward = True

    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    while vel_msg.linear.x != 0:

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
            velocity_publisher.publish(vel_msg)

            t1=rospy.Time.now().to_sec()

            current_distance= speed*(t1-t0)
            print('current_distance=', current_distance)


        vel_msg.linear.x = 0

        velocity_publisher.publish(vel_msg)
    

    image_topic = "/the_robot_camera/camera1/image_raw"

    rospy.Subscriber(image_topic, Image, image_callback)

    #rospy.Subscriber('/the_robot/laser_scan', LaserScan, clbk_laser)

    rospy.spin()

       




if __name__ == '__main__':
    try:
        #Testing our function
        #move(1, 2.5, True)
        

        #move()

        #main()

        moveTemp()



        #main(sys.argv)
        #pass
    except rospy.ROSInterruptException: pass
