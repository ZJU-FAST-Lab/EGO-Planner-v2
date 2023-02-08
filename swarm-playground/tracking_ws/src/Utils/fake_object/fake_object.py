#!/usr/bin/env python3 
import rospy
import tf
import pygame
import random
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    # init pygame

    rospy.init_node('fake_object')
    odom_pub = rospy.Publisher("/object_odom", Odometry, queue_size=100)

    pygame.init()
    pygame.joystick.init()

    INIT_X = 0
    INIT_Y = 0
    INIT_Z = 1.0

    x = INIT_X
    y = INIT_Y
    z = INIT_Z
    yaw = 0
    t = rospy.Time.now()
    last_x = INIT_X
    last_y = INIT_Y
    last_z = INIT_Z
    last_t = rospy.Time.now()
    rate = rospy.Rate(10) # 50hz
    count = 0
    while not rospy.is_shutdown():
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        joystick_count = pygame.joystick.get_count()   
         
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            br = tf.TransformBroadcaster()
            br.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, 0, yaw), rospy.Time.now(), "my_view", "world")

            rate.sleep()  
            
            x -= joystick.get_axis(4) / 5
            y -= joystick.get_axis(3) / 5
            z -= joystick.get_axis(1) / 5
            yaw -= joystick.get_axis(0) / 3  
            
            t = rospy.Time.now()
            send_Od_data = Odometry()
            send_Od_data.header.stamp = t
            send_Od_data.header.frame_id = "world"
            send_Od_data.pose.pose.position.x = x + (random.random() - 0.5) / 2
            send_Od_data.pose.pose.position.y = y + (random.random() - 0.5) / 2
            send_Od_data.pose.pose.position.z = z + (random.random() - 0.5) / 10
            send_Od_data.twist.twist.linear.x = (x + (random.random() - 0.5) / 2 - last_x) / (t-last_t).to_sec()
            send_Od_data.twist.twist.linear.y = (y + (random.random() - 0.5) / 2 - last_y) / (t-last_t).to_sec()
            send_Od_data.twist.twist.linear.z = (z + (random.random() - 0.5) / 10 - last_z) / (t-last_t).to_sec()
            odom_pub.publish(send_Od_data)

            last_x = x
            last_y = y
            last_z = z
            last_t = t

            # print(count)
            # count += 1
            if joystick.get_button(0):
                x = INIT_X
                y = INIT_Y
                z = INIT_Z
                yaw = 0
    
    pygame.quit ()

#      ^                ^
#      |                |
# <-      +0->     <-      +3->        
#    +1|              +4|
#      V                V
