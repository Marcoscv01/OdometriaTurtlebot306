#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

umbral = 0.3 #distancia a la que pararse, medido desde el lidar del robot
r = 0.05 #radio del robot, para calcular el angulo de ataque del lider

n = math.ceil(math.acos(umbral/math.sqrt(umbral**2+r**2))*180/math.pi) #redondeo por arriba para poder iterar las n medidas
l_medida = []
l_umbral = []

for i in range(n):
    l_umbral.append(umbral/math.cos(math.radians(i)))
    print(l_umbral)
l_umbral = l_umbral + l_umbral[::-1]


def detecta(msg):
    global l_medida
    l_medida = msg.ranges[:n] + msg.ranges[-n:]



def stop():
    rospy.init_node("stop_obs")

    rospy.Subscriber("/scan", LaserScan, detecta)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)
    rospy.sleep(1)
    rospy.loginfo("N:%f"%(n))


    while not rospy.is_shutdown():
        t = Twist()
        
        for i in range(n):
            if l_medida[i] < 0.05 or l_medida[i] > l_umbral[i]:
                t.linear.x = 0.05
            else:
                t.linear.x = 0.0
                rospy.loginfo("Objeto detectado")
                rospy.loginfo("Medidas: Real:%f | Umbral: %f"%(l_medida[i],l_umbral[i]))
                break
                    
            

        pub.publish(t)
        rate.sleep()


if __name__ == "__main__":
    stop()