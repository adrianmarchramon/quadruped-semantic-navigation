#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def test_movement():
    rospy.init_node('go1_hardware_test', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 20 Hz es la frecuencia ideal para que el watchdog del perro real no corte la señal
    rate = rospy.Rate(20) 

    rospy.loginfo("🚀 INICIANDO TEST DE HARDWARE GO1")
    rospy.loginfo("⚠️ ATENCIÓN: Asegúrate de que el perro está en el arnés o en el suelo despejado.")
    rospy.sleep(2) # Pausa de seguridad antes de arrancar

    # === 1. AVANZAR ===
    rospy.loginfo("➡️ Avanzando...")
    twist_fwd = Twist()
    twist_fwd.linear.x = 0.15  # Velocidad suave (0.15 m/s)
    
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 3.0 and not rospy.is_shutdown():
        vel_pub.publish(twist_fwd)
        rate.sleep()

    # === 2. PARAR ===
    rospy.loginfo("🛑 Parando...")
    twist_stop = Twist()
    
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 2.0 and not rospy.is_shutdown():
        vel_pub.publish(twist_stop)
        rate.sleep()

    # === 3. RETROCEDER ===
    rospy.loginfo("⬅️ Retrocediendo...")
    twist_bwd = Twist()
    twist_bwd.linear.x = -0.15 # Marcha atrás suave
    
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 3.0 and not rospy.is_shutdown():
        vel_pub.publish(twist_bwd)
        rate.sleep()

    # === 4. PARADA SEGURA FINAL ===
    rospy.loginfo("🏁 Test completado. Apagando motores...")
    # Publicamos la parada varias veces para asegurar que el hardware recibe el 0 absoluto
    for _ in range(10):
        vel_pub.publish(twist_stop)
        rate.sleep()

if __name__ == '__main__':
    try:
        test_movement()
    except rospy.ROSInterruptException:
        pass
