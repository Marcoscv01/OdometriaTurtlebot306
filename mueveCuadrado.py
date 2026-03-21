#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def draw_square():
    rospy.init_node("square_open_loop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(1)

    linear_speed = 0.1       
    angular_speed = 0.5      
    side_length = 0.5         
    turn_angle = 1.5708      

    def move(duration, linear=0.0, angular=0.0):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        end_time = rospy.Time.now().to_sec() + duration
        while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
            pub.publish(cmd)

        pub.publish(Twist())
        rospy.sleep(0.5)

    
    for _ in range(4):
        move(side_length / linear_speed, linear=linear_speed)
        move(turn_angle / angular_speed, angular=angular_speed)

    rospy.loginfo("Cuadrado completado")

if __name__ == "__main__":
    draw_square()