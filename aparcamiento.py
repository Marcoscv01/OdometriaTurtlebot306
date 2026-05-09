#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

umbral_frontal = 0.3
umbral_lateral = 0.3

r = 0.10

lado = "derecha"   # "derecha" o "izquierda"

vel_lineal = 0.2
vel_angular = 0.5


n_f = math.ceil(
    math.acos(
        umbral_frontal /
        math.sqrt(umbral_frontal**2 + r**2)
    ) * 180 / math.pi
)

l_umbral_f = []

for i in range(n_f):
    l_umbral_f.append(
        umbral_frontal / math.cos(math.radians(i))
    )

l_umbral_f = l_umbral_f + l_umbral_f[::-1]



n_l = math.ceil(
    math.acos(
        umbral_lateral /
        math.sqrt(umbral_lateral**2 + r**2)
    ) * 180 / math.pi
)

l_umbral_l = []

for i in range(n_l):
    l_umbral_l.append(
        umbral_lateral / math.cos(math.radians(i))
    )

l_umbral_l = l_umbral_l + l_umbral_l[::-1]



l_medida = [0] * 360

yaw_actual = 0.0

x_actual = 0.0
y_actual = 0.0

v_linear = 0.0
v_angular = 0.0

estado = "BUSCAR"

yaw_inicio = None

t_inicio = 0

x_inicio_entrada = None
y_inicio_entrada = None

distancia_aparcado = 0.0



def scan_callback(msg):

    global l_medida

    l_medida = msg.ranges



def odom_callback(msg):

    global yaw_actual

    global x_actual
    global y_actual

    global v_linear
    global v_angular

    q = msg.pose.pose.orientation

    x_actual = msg.pose.pose.position.x
    y_actual = msg.pose.pose.position.y

    v_linear = msg.twist.twist.linear.x
    v_angular = msg.twist.twist.angular.z

    (_, _, yaw_actual) = euler_from_quaternion(
        [q.x, q.y, q.z, q.w]
    )



def norm(a):

    return math.atan2(
        math.sin(a),
        math.cos(a)
    )



def distancia(x1, y1, x2, y2):

    return math.sqrt(
        (x2 - x1)**2 +
        (y2 - y1)**2
    )


def debug_estado(nombre_estado):

    rospy.loginfo(
        f"[{nombre_estado}] "
        f"x={x_actual:.2f}  "
        f"y={y_actual:.2f}  "
        f"yaw={math.degrees(yaw_actual):.1f}deg  "
        f"v={v_linear:.2f}m/s  "
        f"w={v_angular:.2f}rad/s"
    )



def parking():

    global estado

    global yaw_inicio

    global t_inicio

    global x_inicio_entrada
    global y_inicio_entrada

    global distancia_aparcado

    rospy.init_node("parking_2umbrales")

    rospy.Subscriber(
        "/scan",
        LaserScan,
        scan_callback
    )

    rospy.Subscriber(
        "/odom",
        Odometry,
        odom_callback
    )

    pub = rospy.Publisher(
        "/cmd_vel",
        Twist,
        queue_size=1
    )

    rate = rospy.Rate(5)

    rospy.sleep(1)

    while not rospy.is_shutdown():

        t = Twist()


        frontal = (
            l_medida[-n_f:] +
            l_medida[:n_f]
        )

        if lado == "derecha":

            lateral = l_medida[
                270 - n_l:
                270 + n_l
            ]

        else:

            lateral = l_medida[
                90 - n_l:
                90 + n_l
            ]


        obstaculo_frontal = False

        for i in range(len(frontal)):

            d = frontal[i]

            if d == 0.0 or math.isinf(d):
                continue

            if d < l_umbral_f[i]:

                obstaculo_frontal = True
                break



        hueco_lateral = True

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

            hueco_lateral = (
                fallos < 0.3 * total
            )

        else:

            hueco_lateral = False


        # BUSCAR

        if estado == "BUSCAR":

            debug_estado("BUSCAR")

            t.linear.x = vel_lineal

            if hueco_lateral:

                rospy.loginfo("Hueco detectado")

                estado = "GIRAR"

                yaw_inicio = None


        # GIRAR

        elif estado == "GIRAR":

            debug_estado("GIRAR")

            t.linear.x = 0.0

            if yaw_inicio is None:
                yaw_inicio = yaw_actual

            if lado == "derecha":
                t.angular.z = -vel_angular
            else:
                t.angular.z = vel_angular

            error = norm(
                yaw_actual - yaw_inicio
            )

            if abs(error) >= math.radians(90):

                estado = "ENTRAR"

                x_inicio_entrada = None
                y_inicio_entrada = None


        # ENTRAR

        elif estado == "ENTRAR":

            debug_estado("ENTRAR")

            t.linear.x = vel_lineal

            if x_inicio_entrada is None:

                x_inicio_entrada = x_actual
                y_inicio_entrada = y_actual

            if obstaculo_frontal:

                distancia_aparcado = distancia(
                    x_inicio_entrada,
                    y_inicio_entrada,
                    x_actual,
                    y_actual
                )

                rospy.loginfo(
                    f"Distancia aparcado: "
                    f"{distancia_aparcado:.2f} m"
                )

                estado = "ESPERAR"

                t_inicio = rospy.Time.now().to_sec()

        # ESPERAR

        elif estado == "ESPERAR":

            debug_estado("ESPERAR")

            if (
                rospy.Time.now().to_sec() - t_inicio
            ) > 2:

                estado = "SALIR"


        # SALIR

        elif estado == "SALIR":

            debug_estado("SALIR")

            t.linear.x = -vel_lineal

            distancia_salida = distancia(
                x_actual,
                y_actual,
                x_inicio_entrada,
                y_inicio_entrada
            )

            rospy.loginfo(
                f"Salida: "
                f"{distancia_salida:.2f} / "
                f"{distancia_aparcado:.2f}"
            )

            if distancia_salida >= distancia_aparcado:

                estado = "REORIENTAR"

                yaw_inicio = None


        # REORIENTAR

        elif estado == "REORIENTAR":

            debug_estado("REORIENTAR")

            t.linear.x = 0.0

            if yaw_inicio is None:
                yaw_inicio = yaw_actual

            if lado == "derecha":
                t.angular.z = vel_angular
            else:
                t.angular.z = -vel_angular

            error = norm(
                yaw_actual - yaw_inicio
            )

            if abs(error) >= math.radians(90):

                estado = "FINAL"

        # FINAL

        elif estado == "FINAL":

            debug_estado("FINAL")

            t.linear.x = vel_lineal
            t.angular.z = 0.0

        pub.publish(t)
        rate.sleep()


if __name__ == "__main__":

    parking()