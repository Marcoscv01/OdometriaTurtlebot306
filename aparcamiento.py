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

lado = "izquierda"  # o "izquierda"

# -------- CALCULO FRONTAL --------
n_f = math.ceil(math.acos(umbral_frontal/math.sqrt(umbral_frontal**2+r**2))*180/math.pi)

l_umbral_f = []
for i in range(n_f):
    l_umbral_f.append(umbral_frontal/math.cos(math.radians(i)))
l_umbral_f = l_umbral_f + l_umbral_f[::-1]

# -------- CALCULO LATERAL --------
n_l = math.ceil(math.acos(umbral_lateral/math.sqrt(umbral_lateral**2+r**2))*180/math.pi)

l_umbral_l = []
for i in range(n_l):
    l_umbral_l.append(umbral_lateral/math.cos(math.radians(i)))
l_umbral_l = l_umbral_l + l_umbral_l[::-1]

# -------- VARIABLES --------
l_medida = [0]*360
yaw_actual = 0.0

estado = "BUSCAR"
yaw_inicio = None
t_inicio = 0

# -------- CALLBACKS --------
def scan_callback(msg):
    global l_medida
    l_medida = msg.ranges

def odom_callback(msg):
    global yaw_actual
    q = msg.pose.pose.orientation
    (_, _, yaw_actual) = euler_from_quaternion([q.x, q.y, q.z, q.w])

# -------- MAIN --------
def parking():
    global estado, yaw_inicio, t_inicio

    rospy.init_node("parking_2umbrales")

    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10)
    rospy.sleep(1)

    while not rospy.is_shutdown():

        t = Twist()

        # -------- ABANICOS --------

        frontal = l_medida[-n_f:] + l_medida[:n_f]

        if lado == "derecha":
            lateral = l_medida[270-n_l:270+n_l]
        else:
            lateral = l_medida[90-n_l:90+n_l]

        # -------- DETECCION --------

        obstaculo_frontal = False
        hueco_lateral = True

        # --- FRONTAL ---
        for i in range(len(frontal)):
            d = frontal[i]

            if d == 0.0 or math.isinf(d):
                continue

            if d < l_umbral_f[i]:
                obstaculo_frontal = True
                break

        # --- LATERAL ---
        fallos = 0
        total = 0

        for i in range(len(lateral)):
            d = lateral[i]

            if d == 0.0 or math.isinf(d):
                continue

            total += 1

            if d < l_umbral_l[i]:
                fallos += 1

        if total > 0:
            hueco_lateral = fallos < 0.3 * total
        else:
            hueco_lateral = False

        # -------- NORMALIZAR ANGULO --------
        def norm(a):
            return math.atan2(math.sin(a), math.cos(a))

        # -------- MAQUINA DE ESTADOS --------

        if estado == "BUSCAR":
            rospy.loginfo("Avanzando")

            t.linear.x = 0.2

            if hueco_lateral:
                rospy.loginfo("Hueco detectado")
                estado = "GIRAR"
                yaw_inicio = None

        elif estado == "GIRAR":
            rospy.loginfo("Girando")
            t.linear.x = 0

            if yaw_inicio is None:
                yaw_inicio = yaw_actual

            t.angular.z = -0.5 if lado == "derecha" else 0.5

            error = norm(yaw_actual - yaw_inicio)

            if abs(error) >= math.radians(90):
                estado = "ENTRAR"

        elif estado == "ENTRAR":
            rospy.loginfo("Aparcando")

            t.linear.x = 0.2

            if obstaculo_frontal:
                estado = "ESPERAR"
                t_inicio = rospy.Time.now().to_sec()

        elif estado == "ESPERAR":

            if rospy.Time.now().to_sec() - t_inicio > 2:
                estado = "SALIR"

        elif estado == "SALIR":
            rospy.loginfo("Saliendo")

            t.linear.x = -0.2

            if not obstaculo_frontal:
                estado = "REORIENTAR"
                yaw_inicio = None

        elif estado == "REORIENTAR":
            rospy.loginfo("Girnado")
            t.linear.x = 0.0

            if yaw_inicio is None:
                yaw_inicio = yaw_actual

            t.angular.z = 0.5 if lado == "derecha" else -0.5

            error = norm(yaw_actual - yaw_inicio)

            if abs(error) >= math.radians(90):
                t.linear.x = 0.2
                t.angular.z = 0.0
                t_inicio = rospy.Time.now().to_sec()
                if rospy.Time.now().to_sec() - t_inicio > 2:
                    estado = "BUSCAR"

        pub.publish(t)
        rate.sleep()


if __name__ == "__main__":
    parking()
