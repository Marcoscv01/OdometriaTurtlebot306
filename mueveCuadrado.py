#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

# Parámetros del robot
LINEAR_SPEED = 0.1      # m/s (velocidad hacia adelante)
ANGULAR_SPEED = 0.5     # rad/s (velocidad de giro)

SIDE_LENGTH = 0.5       # metros
TURN_ANGLE = 1.5708     # 90° en radianes = pi/2 rad

def move_forward(pub, distance, speed):
    duration = distance / speed
    command = Twist()
    command.linear.x = speed

    end_time = rospy.Time.now().to_sec() + duration
    while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
        pub.publish(command)

    stop(pub)

def turn(pub, angle, angular_speed):
    duration = angle / angular_speed
    command = Twist()
    command.angular.z = angular_speed

    end_time = rospy.Time.now().to_sec() + duration
    while rospy.Time.now().to_sec() < end_time and not rospy.is_shutdown():
        pub.publish(command)

    stop(pub)

def stop(pub):
    pub.publish(Twist())  # Para el robot
    rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("square_open_loop")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(1)  # Espera a que el publisher esté listo

    for _ in range(4):
        move_forward(pub, SIDE_LENGTH, LINEAR_SPEED)
        turn(pub, TURN_ANGLE, ANGULAR_SPEED)

    rospy.loginfo("Cuadrado completado")