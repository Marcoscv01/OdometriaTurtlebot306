#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

def cuadrado():
    x=0; y=0; yaw=0

    def cb(msg):
        nonlocal x,y,yaw
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_,_,yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

    rospy.init_node("cuadrado")
    rospy.Subscriber("/odom", Odometry, cb)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    for i in range(4):  
        sx = x; sy = y

        while not rospy.is_shutdown():
            d = math.sqrt((x - sx)**2 + (y - sy)**2)
            if 0.5 - d < 0.01:
                break
            t = Twist()
            t.linear.x = 0.5 * (0.5 - d)
            pub.publish(t)
            rate.sleep()

        pub.publish(Twist())

        # Giro de 90 grados
        target = yaw + math.pi/2
        target = math.atan2(math.sin(target), math.cos(target))

        while not rospy.is_shutdown():
            e = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
            if abs(e) < 0.01:
                break
            t = Twist()
            t.angular.z = 1.5 * e
            pub.publish(t)
            rate.sleep()

        pub.publish(Twist())

    rospy.loginfo("Cuadrado completado")

if __name__ == "__main__":
    cuadrado()