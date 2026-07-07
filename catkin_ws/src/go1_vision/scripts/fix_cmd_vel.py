#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from unitree_legged_msgs.msg import MotorCmd

class Go1Fix:
    def __init__(self):
        # Al crear este suscriptor, el tópico /cmd_vel APARECERÁ en la lista
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        
        # Publicamos a los motores de las caderas para que el perro gire
        self.pub_l = rospy.Publisher('/go1_gazebo/FL_hip_controller/command', MotorCmd, queue_size=10)
        self.pub_r = rospy.Publisher('/go1_gazebo/FR_hip_controller/command', MotorCmd, queue_size=10)
        
        rospy.loginfo("--- FIX: Tópico /cmd_vel ACTIVADO ---")

    def callback(self, msg):
        # Creamos el mensaje de motor que el Go1 SÍ entiende
        m_cmd = MotorCmd()
        m_cmd.mode = 1
        m_cmd.Kp = 1.0
        m_cmd.Kd = 0.1
        
        # NOTA: Gira el robot usando cmd_vel (modo 2) en lugar de motor_cmd.
        # Deshabilitado el strafing con la cadera para poder pivotar normalmente.
        if abs(msg.angular.z) > 0.1:
            m_cmd.q = 0.0
            self.pub_l.publish(m_cmd)
            self.pub_r.publish(m_cmd)

if __name__ == '__main__':
    rospy.init_node('go1_cmd_vel_fixer')
    Go1Fix()
    rospy.spin()
