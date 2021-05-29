#!/usr/bin/python
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float64
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
import math

#brouillon
#on a fait ce programme en s'aidant des liens suivant:
#
#https://stackoverflow.com/questions/59131771/how-to-read-a-frame-raw-image-from-a-virtual-turtlebots-camera-in-gazebo-and
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
#https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/
#https://answers.gazebosim.org//question/3336/change-color-of-models/
#http://gazebosim.org/tutorials?tut=build_world
#https://www.semicolonworld.com/question/57841/how-to-find-the-red-color-regions-using-opencv
#https://answers.opencv.org/question/90047/detecting-blue-color-in-this-image/

####!/usr/bin/env python


global shutdown_var
global couleur_bleue_trouvee
global else_active


deploiementDuGripperEnModeChasse = 0


x = 0.0
y = 0.0 
theta = 0.0




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
            vel_msg.linear.x =  0.2;

            #vel_msg.angular.z -= 1

            print('vel_msg.angular.z=', vel_msg.angular.z)

            print("DE NOUVEAU")



            print('DANS IF >50 z angular=', vel_msg.angular.z)

            velocity_publisher.publish(vel_msg)
            #couleur_bleue_trouvee = 1
        """
        else:
            if couleur_bleue_trouvee == 1:
                #avance_robot_vers_devant(5)

                #print('YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY')

                vel_msg.linear.x = 0.2
                vel_msg.angular.z = 0.0
                
                couleur_bleue_trouvee = 0
                velocity_publisher.publish(vel_msg)
        """
        """
        if vel_msg.angular.z == 0:
            print("ANGULAR 00000")
            couleur_bleue_trouvee = 1
            #rospy.signal_shutdown("end /the_robot/cmd_vel")
        """

        """
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
       """




def take_action(regions):
    global shutdown_var
    global deploiementDuGripperEnModeChasse

    msg = Twist()
    linear_x = 0
    angular_z = 0
    change_velocity = 0

    velocity_publisher = rospy.Publisher('/the_robot/cmd_vel', Twist, queue_size=10)

    state_description = ''

    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 1 - nothing'
        print("DANS LE CAS 1")
        #linear_x = 0.6
        #angular_z = 0
        #print("msg_t.ranges[360]=", msg_t.ranges[360])

    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        print("DANS LE CAS 2")
        #linear_x = 0
        #angular_z = -0.3

    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        print("DANS LE CAS 3")
        #linear_x = 0
        #angular_z = -0.3

    elif regions['front'] > 1 and regions['fleft'] < 2 and regions['fright'] > 2:
        state_description = 'case 4 - fleft'
        print("DANS LE CAS 4")
        linear_x = 0
        angular_z = 0.3
        change_velocity = 1

    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        print("DANS LE CAS 5")
        #linear_x = 0
        #angular_z = -0.3

    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        print("DANS LE CAS 6")
        #linear_x = 0
        #angular_z = 0.3

    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        print("DANS LE CAS 7")
        #shutdown_var = 1
        #rospy.shu
        linear_x = 0
        angular_z = 0
        change_velocity = 1
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        print("DANS LE CAS 8")
        #shutdown_var = 1
        #linear_x = 0
        #angular_z = 0
        change_velocity = 1
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    if change_velocity == 1:
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        velocity_publisher.publish(msg)
        deploiementDuGripperEnModeChasse = 1
        #change_velocity = 0





def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }

    take_action(regions)



def avance_robot_vers_devant(distance):

            velocity_publisher = rospy.Publisher('/the_robot/cmd_vel', Twist, queue_size=10)

            vel_msg = Twist();

            speed = 5
            distanceN = 50
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

                while(current_distance < distanceN):
                    velocity_publisher.publish(vel_msg)

                    t1=rospy.Time.now().to_sec()
                    current_distance= speed*(t1-t0)


                vel_msg.linear.x = 0
                velocity_publisher.publish(vel_msg)

            
            print("FINDUWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW")




def avancer_devant():

    print("AVANT ODOMETRY")
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    pub = rospy.Publisher("/the_robot/cmd_vel", Twist, queue_size = 1)

    speed = Twist()

    r = rospy.Rate(4)

    goal = Point()
    goal.x = -40
    goal.y = -40

    while not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            print("IIIIIF")
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            print("ELSEEEEEEE")
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()  



def deploiementDuGripperEnModeChasse():
    gripper_left_finger_publisher = rospy.Publisher('/the_robot__palm_left_finger_position_controller/command', Float64, queue_size=10)
    gripper_left_finger_tip_publisher = rospy.Publisher('/the_robot__left_finger_tip_joint_position_controller/command', Float64, queue_size=10)
    gripper_right_finger_publisher = rospy.Publisher('/the_robot__palm_right_finger_position_controller/command', Float64, queue_size=10)
    gripper_right_finger_tip_publisher = rospy.Publisher('/the_robot__right_finger_tip_joint_position_controller/command', Float64, queue_size=10)
    gripper_palm_riser_pusblisher = rospy.Publisher('/the_robot__palm_riser_position_controller/command', Float64, queue_size=10)


    rate = rospy.Rate(10) # 10hz
    end_loop = 0
    i = 0;
    while i < 0.8:
      msg_data_palm_riser = 5.0
      msg_data_left_finger = i;
      gripper_palm_riser_pusblisher.publish(msg_data_palm_riser)
      gripper_left_finger_publisher.publish(msg_data_left_finger)
      gripper_left_finger_tip_publisher.publish(msg_data_left_finger)
      gripper_right_finger_publisher.publish(-msg_data_left_finger)
      gripper_right_finger_tip_publisher.publish(-msg_data_left_finger)
      rospy.loginfo(msg_data_left_finger)
      rate.sleep()
      i += 0.01
      end_loop += 1

def researchObjectAndCatchWithGripper(msg):
    gripper_left_finger_publisher = rospy.Publisher('/the_robot__palm_left_finger_position_controller/command', Float64, queue_size=10)
    gripper_left_finger_tip_publisher = rospy.Publisher('/the_robot__left_finger_tip_joint_position_controller/command', Float64, queue_size=10)
    gripper_right_finger_publisher = rospy.Publisher('/the_robot__palm_right_finger_position_controller/command', Float64, queue_size=10)
    gripper_right_finger_tip_publisher = rospy.Publisher('/the_robot__right_finger_tip_joint_position_controller/command', Float64, queue_size=10)
    gripper_palm_riser_pusblisher = rospy.Publisher('/the_robot__palm_riser_position_controller/command', Float64, queue_size=10)

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

        print("dans la callback researchObjectAndCatchWithGripper")
        print("type of msg=", type(msg))

        rate = rospy.Rate(10) # 10hz
        end_loop = 0
        i = 0;
        while i < 0.8:
          msg_data_palm_riser = 5.0
          msg_data_left_finger = i;
          gripper_palm_riser_pusblisher.publish(msg_data_palm_riser)
          gripper_left_finger_publisher.publish(-msg_data_left_finger)
          gripper_left_finger_tip_publisher.publish(-msg_data_left_finger)
          gripper_right_finger_publisher.publish(msg_data_left_finger)
          gripper_right_finger_tip_publisher.publish(msg_data_left_finger)
          rospy.loginfo(msg_data_left_finger)
          rate.sleep()
          i += 0.01
          end_loop += 1


        avance_robot_vers_devant(5)

      
def newOdom(msg):
    global x
    global y
    global theta

    print("NEWODOM")

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    print("xNEWODOM=", x)
    print("yNEWODOM=", y)

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def moveTemp():
    global shutdown_var
    global couleur_bleue_trouvee
    global else_active
    global deploiementDuGripperEnModeChasse

    shutdown_var = 0
    couleur_bleue_trouvee = 0
    else_active = 0

    
    rospy.init_node('robot_search_blue_color', anonymous=True)

    velocity_publisher = rospy.Publisher('/the_robot/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    

    print("dans movetemp")
    

    #avance_robot_vers_devant(1)



    if deploiementDuGripperEnModeChasse == 1:
        deploiementDuGripperEnModeChasse()


    """
    rate = rospy.Rate(10) # 10hz
    end_loop = 0
    i = 0;
    while end_loop < 15: #  i < 1.5: # not rospy.is_shutdown():
      msg_data_palm_riser = 10.8
      msg_data_left_finger = i;
      gripper_palm_riser_pusblisher.publish(msg_data_palm_riser)
      gripper_left_finger_publisher.publish(msg_data_left_finger)
      rospy.loginfo(msg_data_left_finger)
      rate.sleep()
      i += 0.01
      end_loop += 1
    """



    """
    speed = 10
    distance = 100
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

        while(current_distance < distance*distance):
            velocity_publisher.publish(vel_msg)

            t1=rospy.Time.now().to_sec()

            current_distance= speed*(t1-t0)
            print('current_distance=', current_distance)


        vel_msg.linear.x = 0

        velocity_publisher.publish(vel_msg)
    
    """

    """
    print("AVANT ODOMETRY")
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    pub = rospy.Publisher("/the_robot/cmd_vel", Twist, queue_size = 1)

    speed = Twist()

    r = rospy.Rate(4)

    goal = Point()
    goal.x = -3
    goal.y = 1

    while math.floor(x) != goal.x and math.floor(y) != goal.y:  #not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)

        print("ANGLE TO GOAAAAL")

        print('math.floor(x)=', math.floor(x))
        print('math.floor(y)=', math.floor(y))
        print('x=', x)
        print('y=', y)
        print('goal.x=', goal.x)
        print('goal.y=', goal.y)

        if abs(angle_to_goal - theta) > 0.1:
            print("IIIIIF")
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            print("ELSEEEEEEE")
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()

    speed.linear.x = 0.0
    speed.angular.z = 0.0
    
    pub.publish(speed)
    """



    image_topic = "/the_robot_camera/camera1/image_raw"
    

    gripper_topic = "/the_robot__palm_left_finger_position_controller/command"
    

    #researchObjectAndCatchWithGripper(Image);
    #rospy.Subscriber(image_topic, Image, researchObjectAndCatchWithGripper)

    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.Subscriber('/the_robot/laser_scan', LaserScan, clbk_laser)
    
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
