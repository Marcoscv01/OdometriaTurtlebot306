#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

# -------- PARAMETROS --------
umbral_frontal = 0.3
umbral_lateral = 0.3
r = 0.10
lado = "izquierda"

# Parámetros del Regulador Proporcional
Kp_angular = 1.2  
Kp_linear = 0.5   
v_max = 0.2       
w_max = 0.6       

# -------- CALCULO UMBRALES --------
n_f = math.ceil(math.acos(umbral_frontal / math.sqrt(umbral_frontal**2 + r**2)) * 180 / math.pi)
l_umbral_f = [umbral_frontal / math.cos(math.radians(i)) for i in range(n_f)]
l_umbral_f = l_umbral_f + l_umbral_f[::-1]

n_l = math.ceil(math.acos(umbral_lateral / math.sqrt(umbral_lateral**2 + r**2)) * 180 / math.pi)
l_umbral_l = [umbral_lateral / math.cos(math.radians(i)) for i in range(n_l)]
l_umbral_l = l_umbral_l + l_umbral_l[::-1]

# -------- VARIABLES --------
l_medida = [0] * 360
yaw_actual = 0.0
pos_actual = [0.0, 0.0]
estado = "BUSCAR"
yaw_inicio = None
t_inicio = 0
pos_inicio_entrar = [0.0, 0.0]
distancia_recorrida_entrar = 0.0
pos_inicio_salir = [0.0, 0.0]

# -------- CALLBACKS --------
def scan_callback(msg):
    global l_medida
    l_medida = msg.ranges

def odom_callback(msg):
    global yaw_actual, pos_actual
    pos_actual = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    q = msg.pose.pose.orientation
    (_, _, yaw_actual) = euler_from_quaternion([q.x, q.y, q.z, q.w])

def norm(a):
    return math.atan2(math.sin(a), math.cos(a))

# -------- MAIN --------
def parking():
    global estado, yaw_inicio, t_inicio, pos_inicio_entrar, distancia_recorrida_entrar, pos_inicio_salir

    rospy.init_node("parking_proporcional")
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(10)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        t = Twist()
        
        # --- ABANICOS ---
        frontal = l_medida[-n_f:] + l_medida[:n_f]
        lateral = l_medida[270 - n_l:270 + n_l] if lado == "derecha" else l_medida[90 - n_l:90 + n_l]
        
        obstaculo_frontal = any(d < l_umbral_f[i] for i, d in enumerate(frontal) if d > 0.0 and not math.isinf(d))
        
       
        if estado == "BUSCAR":
            t.linear.x = v_max
            if not any(d < l_umbral_l[i] for i, d in enumerate(lateral) if d > 0.0 and not math.isinf(d)):
                estado = "GIRAR"
                yaw_inicio = None

        elif estado == "GIRAR":
            if yaw_inicio is None: yaw_inicio = yaw_actual
            
            objetivo = norm(yaw_inicio + (math.radians(-90) if lado == "derecha" else math.radians(90)))
            error_angular = norm(objetivo - yaw_actual)
            
            t.angular.z = Kp_angular * error_angular
            
            t.angular.z = max(min(t.angular.z, w_max), -w_max)

            if abs(error_angular) < 0.05: 
                estado = "ENTRAR"
                pos_inicio_entrar = [pos_actual[0], pos_actual[1]]

        elif estado == "ENTRAR":
            t.angular.z = 0.0
            t.linear.x = v_max
            if obstaculo_frontal:
                distancia_recorrida_entrar = math.sqrt((pos_actual[0]-pos_inicio_entrar[0])**2 + (pos_actual[1]-pos_inicio_entrar[1])**2)
                estado = "ESPERAR"
                t_inicio = rospy.Time.now().to_sec()

        elif estado == "ESPERAR":
            if rospy.Time.now().to_sec() - t_inicio > 2:
                estado = "SALIR"
                pos_inicio_salir = [pos_actual[0], pos_actual[1]]

        elif estado == "SALIR":
            distancia_actual = math.sqrt((pos_actual[0]-pos_inicio_salir[0])**2 + (pos_actual[1]-pos_inicio_salir[1])**2)
            error_distancia = distancia_recorrida_entrar - distancia_actual
            
            
            t.linear.x = -max(min(Kp_linear * abs(error_distancia),v_max), 0.05) 
            
            if error_distancia <= 0.01: 
                estado = "REORIENTAR"
                yaw_inicio = None

        elif estado == "REORIENTAR":
            if yaw_inicio is None: yaw_inicio = yaw_actual
            
            objetivo = norm(yaw_inicio + (math.radians(90) if lado == "derecha" else math.radians(-90)))
            error_angular = norm(objetivo - yaw_actual)
            
            t.angular.z = Kp_angular * error_angular
            t.angular.z = max(min(t.angular.z, w_max), -w_max)

            if abs(error_angular) < 0.05:
                estado = "FINAL"

        elif estado == "FINAL":
            t.linear.x = v_max

       
        rospy.loginfo(f"ESTADO: {estado} | ODOM: [{pos_actual[0]:.2f}, {pos_actual[1]:.2f}] Yaw: {yaw_actual:.2f} | CMD: v={t.linear.x:.2f} w={t.angular.z:.2f}")

        pub.publish(t)
        rate.sleep()

if __name__ == "__main__":
    parking()