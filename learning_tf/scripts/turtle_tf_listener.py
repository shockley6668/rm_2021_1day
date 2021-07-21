#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_service::Person

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
def to_euler_angles(x, y, z, w):
    """w、x、y、z to euler angles"""
    angles = []
    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*z))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    
    angles.append(r)
    angles.append(p)
    angles.append(y)
    return angles
if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(5, 5, 0, 'turtle2')
    spawner(5, 6, 0, 'turtle3')
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    turtle_vel3 = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    
    # (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))

    
    rate = rospy.Rate(10.0)
    a=0
    ang=[]
    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            (trans3,rot3) = listener.lookupTransform('/turtle3', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # rospy.loginfo(rot)
        if a==0:
            bx=trans[0]
            by=trans[1]
            bx3=trans3[0]
            by3=trans3[1]
        a=1
        ang=to_euler_angles(rot[0],rot[1],rot[2],rot[3])
        ang3=to_euler_angles(rot3[0],rot3[1],rot3[2],rot3[3])
        
        yaw=ang[2]
        yaw3=ang3[2]
        
        cx=trans[0]-bx
        cy=trans[1]-by
        
        cx3=trans3[0]-bx3
        cy3=trans3[1]-by3
        
        cmd = geometry_msgs.msg.Twist()
        cmd3 = geometry_msgs.msg.Twist()
        
        cmd.linear.x = cx*10
        cmd.linear.y = cy*10
        cmd.angular.z = yaw*10

        cmd3.linear.x = cx3*10
        cmd3.linear.y = cy3*10
        cmd3.angular.z = yaw3*10
        
        turtle_vel.publish(cmd)
        turtle_vel3.publish(cmd3)

        rate.sleep()


